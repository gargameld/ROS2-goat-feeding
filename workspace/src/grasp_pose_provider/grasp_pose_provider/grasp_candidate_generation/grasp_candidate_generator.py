"""Generation of the grasp candidates the provider node offers.

One of the two parts :mod:`grasp_pose_provider.node_initializer` builds the
node from -- the other being
:class:`grasp_pose_provider.grasp_candidate_validator.GraspCandidateValidator`.
:class:`GraspCandidateGenerator` ties the rest of this subpackage together into
a single call: capture a cloud, detect the food, ask the GPD service for grasp
candidates, and hand back a ranked list of TCP grasp poses. Reporting those
candidates to RViz is
:class:`grasp_pose_provider.grasp_candidate_publisher.GraspCandidatePublisher`'s
job, not this one's.

:meth:`GraspCandidateGenerator.generate` runs the pipeline:

1. ``capture_combined_cloud`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.combine_pointclouds`
   waits for one ``sensor_msgs/msg/PointCloud2`` on each of the three camera
   topics and patches them into a single cloud in the reference camera's frame,
   using
   :mod:`grasp_pose_provider.grasp_candidate_generation.camera_transforms` for
   the camera-to-camera transforms.
2. ``load_stored_model`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.stored_model`
   reads the three stored empty-plate dumps -- one per camera -- and merges
   them into the same reference frame.
3. ``detect_food`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.food_detector`
   -> food-point indices.
3b. ``FoodCloudPublisher`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.food_cloud_publisher`
   republishes those points on their own, as the single cloud MoveIt builds
   the planning-scene octomap from.
4. ``build_service_request`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.gpd_request_builder`
   -> a ``DetectConstrainedGrasps`` request.
5. Call the ``detect_constrained_grasps`` service.
6. ``grasp_configs_to_poses`` in
   :mod:`grasp_pose_provider.grasp_candidate_generation.grasp_config_conversion`
   -> TCP grasp poses in ``base_link``.

Both merges target the reference camera's frame -- ``left_camera_frame`` by
default -- so the model and the scene are directly comparable. The final poses
are expressed in ``base_link`` via the camera transforms published from the
robot description.

Progress is reported through an optional ``feedback_cb`` -- a callable taking a
single stage string -- so an action server can forward it as action feedback.

The generator owns the long-lived helpers the pipeline needs -- the tf2-backed
transform resolver, the point cloud snapshotter and the food cloud publisher --
because each of them has to be up well before the first goal arrives rather
than being created per goal.

The node must be spun by an executor (e.g. the one in
:mod:`grasp_pose_provider.main`) for the duration of :meth:`generate`: the GPD
service response is delivered by that executor.
"""

import time

from gpd_ros2_msgs.srv import DetectConstrainedGrasps
from grasp_pose_provider.grasp_candidate_generation import (
    camera_transforms,
    combine_pointclouds,
    food_cloud_publisher as food_cloud,
    food_detector,
    gpd_request_builder,
    grasp_config_conversion,
    stored_model,
)
import rclpy


# Re-exported so callers of :meth:`GraspCandidateGenerator.generate` can catch
# the empty-plate case without reaching into the food modules themselves.
NoFoodDetectedError = food_detector.NoFoodDetectedError


def _emit(feedback_cb, stage):
    """Forward a progress ``stage`` to ``feedback_cb`` if one was provided."""
    if feedback_cb is not None:
        feedback_cb(stage)


class GraspCandidateGenerator:
    """Produce ranked TCP grasp candidates for the current scene."""

    def __init__(self, node, parameters, callback_group=None):
        """Create the generator's snapshotter, tf2 listener and food cloud.

        ``parameters`` is the node's already-populated
        :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`;
        every value the pipeline needs is read from it, so nothing else has to
        be passed in. ``callback_group`` should be reentrant so the executor
        can keep delivering point clouds and the GPD service response while
        :meth:`generate` waits on them.
        """
        self._node = node
        self._parameters = parameters

        # Created once and kept: the occupancy map monitor subscribes when
        # move_group starts, so a per-goal publisher could lose the single
        # cloud each cycle sends to discovery latency.
        self._food_cloud_publisher = food_cloud.FoodCloudPublisher(
            node, parameters
        )

        # Kept for the node's lifetime: the tf2 buffer only answers lookups for
        # transforms it has already received, so the listener has to be up long
        # before the first goal arrives.
        self._transform_resolver = (
            camera_transforms.CameraTransformResolver(node, parameters)
        )

        self._pointcloud_snapshotter = (
            combine_pointclouds.PointCloudSnapshotter(
                node, parameters, callback_group=callback_group
            )
        )

    def generate(self, feedback_cb=None, snapshot_captured_cb=None):
        """Capture the camera clouds and return candidate TCP grasp poses.

        Returns a list of ``geometry_msgs/msg/PoseStamped`` -- one per grasp
        candidate the GPD service returned, in GPD's own score order --
        stamped in ``base_link``. Raises :class:`NoFoodDetectedError` when
        segmentation finds too little food to grasp.

        ``feedback_cb``, if given, is called with a short stage string at each
        step of the pipeline. ``snapshot_captured_cb`` is called as soon as one
        fresh message from every camera has been captured.
        """
        parameters = self._parameters

        initial_sequences = self._pointcloud_snapshotter.mark()
        combined = combine_pointclouds.capture_combined_cloud(
            self._node,
            parameters,
            self._transform_resolver,
            feedback_cb=feedback_cb,
            snapshotter=self._pointcloud_snapshotter,
            after_sequences=initial_sequences,
            snapshot_captured_cb=snapshot_captured_cb,
        )

        stored_cloud = stored_model.load_stored_model(
            parameters,
            self._transform_resolver,
            reference_frame=combined.frame_id,
            feedback_cb=feedback_cb,
        )

        _emit(feedback_cb, 'Detecting food in the combined cloud')
        base_from_cloud_matrix = (
            self._transform_resolver.lookup_base_from_camera(
                combined.frame_id,
                stamp=combined.msg.header.stamp,
            )
        )
        food_indices = food_detector.detect_food(
            parameters,
            stored_cloud,
            combined.msg,
            base_from_cloud_matrix,
        )
        food_detector.require_minimum_food_points(parameters, food_indices)

        # Publish before the GPD call, not after: the octomap updater resolves
        # the cloud's frame at the cloud's own stamp, and the GPD detection
        # takes long enough (minutes) that the capture stamp would have aged
        # out of move_group's tf2 buffer by the time it returns.
        _emit(
            feedback_cb, 'Publishing the food points for the planning octomap'
        )
        self._food_cloud_publisher.publish(combined.msg, food_indices)

        _emit(feedback_cb, 'Cropping the GPD cloud around the detected food')
        cropped_msg, cropped_food_indices, cropped_camera_source = (
            gpd_request_builder.crop_cloud_around_indices(
                parameters,
                combined.msg,
                food_indices,
                camera_source=combined.camera_source,
            )
        )

        _emit(feedback_cb, 'Building the GPD service request')
        request = gpd_request_builder.build_service_request(
            cropped_food_indices,
            cropped_msg,
            camera_source=cropped_camera_source,
            view_points=combined.view_points,
        )

        _emit(feedback_cb, 'Calling the GPD grasp-detection service')
        grasp_config_list = self._call_gpd_service(request)

        _emit(feedback_cb, 'Converting GPD grasps into base_link poses')
        poses = grasp_config_conversion.grasp_configs_to_poses(
            parameters,
            grasp_config_list,
            target_from_grasp_frame=base_from_cloud_matrix,
        )
        for pose in poses:
            pose.header.stamp = combined.msg.header.stamp

        self._node.get_logger().info(
            f'Returning {len(poses)} grasp candidates in the order GPD '
            'scored them'
        )
        return poses

    def _call_gpd_service(self, request):
        """Call the GPD service and return the response's ``GraspConfigList``.

        The future is awaited without spinning the node here; the executor that
        is already spinning it (see the module docstring) delivers the
        response.
        """
        service_name = self._parameters.gpd_service_name
        timeout_sec = self._parameters.gpd_service_timeout_sec
        client = self._node.create_client(
            DetectConstrainedGrasps, service_name
        )
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(
                    f"GPD service '{service_name}' not available after "
                    f'{timeout_sec}s.'
                )

            future = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while (rclpy.ok() and not future.done()
                   and time.monotonic() < deadline):
                time.sleep(0.01)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    f"GPD service '{service_name}' call failed or timed out."
                )
            return future.result().grasp_configs
        finally:
            self._node.destroy_client(client)

    def destroy(self):
        """Release the helpers this part owns."""
        self._food_cloud_publisher.destroy()
        self._pointcloud_snapshotter.destroy()
        self._transform_resolver.destroy()
