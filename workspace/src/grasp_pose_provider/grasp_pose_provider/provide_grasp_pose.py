"""Top-level grasp pose provider.

Ties the lower-level components together into a single call: capture a cloud,
detect the food, ask the GPD service for grasp candidates, and hand back a
list of TCP grasp poses.

The pipeline:

1. :func:`grasp_pose_provider.combine_pointclouds.capture_combined_cloud` waits
   for one ``sensor_msgs/msg/PointCloud2`` on each of the three camera topics
   and patches them into a single cloud in the reference camera's frame, using
   :mod:`grasp_pose_provider.camera_transforms` for the camera-to-camera
   transforms.
2. :func:`grasp_pose_provider.stored_model.load_stored_model` reads the three
   stored empty-plate dumps -- one per camera -- and merges them into the same
   reference frame.
3. :func:`grasp_pose_provider.food_detector.detect_food` -> food-point indices.
4. :func:`grasp_pose_provider.gpd_request_builder.build_service_request` ->
   a ``DetectConstrainedGrasps`` request.
5. Call the ``detect_constrained_grasps`` service.
6. :func:`grasp_pose_provider.grasp_config_conversion.grasp_configs_to_poses`
   -> a list of TCP grasp poses.

Both merges target the reference camera's frame -- ``left_camera_frame`` by
default -- so the model and the scene are directly comparable, and everything
from step 3 on works exactly as it did on the single-camera clouds.

Progress is reported through an optional ``feedback_cb`` -- a callable taking a
single stage string -- so an action server can forward it as action feedback.

Only the pose list is produced for now; integrating MoveIt to pick a reachable
grasp from that list is left for later.

``node`` must be spun by an executor (e.g. the one in
:mod:`grasp_pose_provider.main`) for the duration of the call: the GPD service
response is delivered by that executor.
"""

import time

import rclpy

from gpd_ros2_msgs.srv import DetectConstrainedGrasps

from grasp_pose_provider import (
    camera_transforms,
    combine_pointclouds,
    food_detector,
    gpd_request_builder,
    grasp_config_conversion,
    stored_model,
)


# Point cloud topics published by the three simulated left-facing cameras. The
# first is the reference camera: the merged cloud comes out in its frame.
DEFAULT_CAPTURED_TOPICS = combine_pointclouds.DEFAULT_CAMERA_TOPICS
# How long to block waiting for a message on each captured topic (seconds).
DEFAULT_WAIT_TIMEOUT_SEC = 10.0
# The GPD grasp-detection service.
GPD_SERVICE_NAME = 'detect_constrained_grasps'
# How long to wait for the GPD service to be available / to answer (seconds).
# A full detection on the merged three-camera cloud has been measured at ~130 s
# end to end, so this has to stay well clear of two minutes.
DEFAULT_SERVICE_TIMEOUT_SEC = 300.0


def _emit(feedback_cb, stage):
    """Forward a progress ``stage`` to ``feedback_cb`` if one was provided."""
    if feedback_cb is not None:
        feedback_cb(stage)


def provide_grasp_pose(
    node,
    stored_pointcloud_dir,
    transform_resolver,
    captured_topics=DEFAULT_CAPTURED_TOPICS,
    wait_timeout_sec=DEFAULT_WAIT_TIMEOUT_SEC,
    service_timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC,
    tf_timeout_sec=camera_transforms.DEFAULT_TF_TIMEOUT_SEC,
    feedback_cb=None,
):
    """Capture the camera clouds and return candidate TCP grasp poses.

    ``stored_pointcloud_dir`` holds the stored empty-plate dump of each camera,
    named after that camera (see
    :func:`grasp_pose_provider.stored_model.stored_pointcloud_paths`).
    ``transform_resolver`` is a
    :class:`~grasp_pose_provider.camera_transforms.CameraTransformResolver`
    kept alive by the caller; it supplies the camera-to-camera transforms both
    merges need.

    Returns a list of ``geometry_msgs/msg/PoseStamped`` -- one per grasp
    candidate returned by the GPD service -- stamped in the merged cloud's
    frame. ``feedback_cb``, if given, is called with a short stage string at
    each step of the pipeline.
    """
    combined = combine_pointclouds.capture_combined_cloud(
        node,
        transform_resolver,
        topics=captured_topics,
        wait_timeout_sec=wait_timeout_sec,
        tf_timeout_sec=tf_timeout_sec,
        feedback_cb=feedback_cb,
    )

    stored_cloud = stored_model.load_stored_model(
        stored_pointcloud_dir,
        transform_resolver,
        reference_frame=combined.frame_id,
        topics=captured_topics,
        tf_timeout_sec=tf_timeout_sec,
        feedback_cb=feedback_cb,
    )

    _emit(feedback_cb, 'Detecting food in the combined cloud')
    food_indices = food_detector.detect_food(stored_cloud, combined.msg)

    _emit(feedback_cb, 'Building the GPD service request')
    request = gpd_request_builder.build_service_request(
        food_indices,
        combined.msg,
        camera_source=combined.camera_source,
        view_points=combined.view_points,
    )

    _emit(feedback_cb, 'Calling the GPD grasp-detection service')
    grasp_config_list = _call_gpd_service(node, request, service_timeout_sec)

    _emit(feedback_cb, 'Converting grasp configurations to poses')
    return grasp_config_conversion.grasp_configs_to_poses(grasp_config_list)


def _call_gpd_service(node, request, service_timeout_sec):
    """Call the GPD service and return the response's ``GraspConfigList``.

    The future is awaited without spinning ``node`` here; the executor that is
    already spinning the node (see the module docstring) delivers the response.
    """
    client = node.create_client(DetectConstrainedGrasps, GPD_SERVICE_NAME)
    try:
        if not client.wait_for_service(timeout_sec=service_timeout_sec):
            raise RuntimeError(
                f"GPD service '{GPD_SERVICE_NAME}' not available after "
                f'{service_timeout_sec}s.'
            )

        future = client.call_async(request)
        deadline = time.monotonic() + service_timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or future.result() is None:
            raise RuntimeError(
                f"GPD service '{GPD_SERVICE_NAME}' call failed or timed out."
            )
        return future.result().grasp_configs
    finally:
        node.destroy_client(client)
