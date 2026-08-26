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
3b. :class:`grasp_pose_provider.food_cloud_publisher.FoodCloudPublisher`
   republishes those points on their own, as the single cloud MoveIt builds
   the planning-scene octomap from.
4. :func:`grasp_pose_provider.gpd_request_builder.build_service_request` ->
   a ``DetectConstrainedGrasps`` request.
5. Call the ``detect_constrained_grasps`` service.
6. :func:`grasp_pose_provider.grasp_config_conversion.grasp_configs_to_poses`
   -> TCP grasp poses in ``base_link``.

Both merges target the reference camera's frame -- ``left_camera_frame`` by
default -- so the model and the scene are directly comparable. The final
poses are expressed in ``base_link`` via the camera transforms published from
the robot description.

Progress is reported through an optional ``feedback_cb`` -- a callable taking a
single stage string -- so an action server can forward it as action feedback.

Only the pose list is produced for now; integrating MoveIt to pick a reachable
grasp from that list is left for later.

``node`` must be spun by an executor (e.g. the one in
:mod:`grasp_pose_provider.main`) for the duration of the call: the GPD service
response is delivered by that executor.
"""

import time

from gpd_ros2_msgs.srv import DetectConstrainedGrasps
from grasp_pose_provider import (
    camera_transforms,
    combine_pointclouds,
    food_cloud_publisher as food_cloud_publisher_module,
    food_detector,
    food_presence,
    gpd_request_builder,
    grasp_config_conversion,
    grasp_pose_ranking,
    stored_model,
)
import rclpy


# Point cloud topics published by the three simulated left-facing cameras. The
# first is the reference camera: the merged cloud comes out in its frame.
DEFAULT_CAPTURED_TOPICS = combine_pointclouds.DEFAULT_CAMERA_TOPICS
# How long to block waiting for a message on each captured topic (seconds).
DEFAULT_WAIT_TIMEOUT_SEC = combine_pointclouds.DEFAULT_WAIT_TIMEOUT_SEC
# The GPD grasp-detection service.
GPD_SERVICE_NAME = 'detect_constrained_grasps'
# How long to wait for the GPD service to be available / to answer (seconds).
# A full detection on the merged three-camera cloud has been measured at ~130 s
# end to end, so this has to stay well clear of two minutes.
DEFAULT_SERVICE_TIMEOUT_SEC = 600.0
# Only local scene geometry is needed to evaluate grasps around the food.
DEFAULT_GPD_CLOUD_CROP_RADIUS = 0.10
DEFAULT_MIN_FOOD_POINT_COUNT = food_presence.DEFAULT_MIN_FOOD_POINT_COUNT
NoFoodDetectedError = food_presence.NoFoodDetectedError


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
    min_food_point_count=DEFAULT_MIN_FOOD_POINT_COUNT,
    tcp_from_finger_base=(
        grasp_config_conversion.DEFAULT_TCP_FROM_FINGER_BASE
    ),
    feedback_cb=None,
    pointcloud_snapshotter=None,
    snapshot_captured_cb=None,
    food_cloud_publisher=None,
):
    """Capture the camera clouds and return candidate TCP grasp poses.

    ``stored_pointcloud_dir`` holds the stored empty-plate dump of each camera,
    named after that camera (see
    :func:`grasp_pose_provider.stored_model.stored_pointcloud_paths`).
    ``transform_resolver`` is a
    :class:`~grasp_pose_provider.camera_transforms.CameraTransformResolver`
    kept alive by the caller; it supplies the camera-to-camera transforms both
    merges need and the merged-camera-to-base transform used to reject shelf
    surfaces.

    Returns a list of ``geometry_msgs/msg/PoseStamped`` -- one per grasp
    candidate returned by the GPD service -- stamped in ``base_link``.
    ``feedback_cb``, if given, is called with a short stage string at each
    step of the pipeline. ``snapshot_captured_cb`` is called as soon as one
    fresh message from every camera has been captured.
    ``food_cloud_publisher`` is a
    :class:`~grasp_pose_provider.food_cloud_publisher.FoodCloudPublisher` kept
    alive by the caller; the detected food points are pushed through it so the
    planning-scene octomap sees them.
    """
    owns_snapshotter = pointcloud_snapshotter is None
    if owns_snapshotter:
        pointcloud_snapshotter = combine_pointclouds.PointCloudSnapshotter(
            node, captured_topics
        )
    owns_food_cloud_publisher = food_cloud_publisher is None
    if owns_food_cloud_publisher:
        food_cloud_publisher = food_cloud_publisher_module.FoodCloudPublisher(
            node
        )
    try:
        return _run_pipeline(
            node=node,
            stored_pointcloud_dir=stored_pointcloud_dir,
            transform_resolver=transform_resolver,
            captured_topics=captured_topics,
            wait_timeout_sec=wait_timeout_sec,
            service_timeout_sec=service_timeout_sec,
            tf_timeout_sec=tf_timeout_sec,
            min_food_point_count=min_food_point_count,
            tcp_from_finger_base=tcp_from_finger_base,
            feedback_cb=feedback_cb,
            pointcloud_snapshotter=pointcloud_snapshotter,
            snapshot_captured_cb=snapshot_captured_cb,
            food_cloud_publisher=food_cloud_publisher,
        )
    finally:
        if owns_snapshotter:
            pointcloud_snapshotter.destroy()
        if owns_food_cloud_publisher:
            food_cloud_publisher.destroy()


def _run_pipeline(
    node,
    stored_pointcloud_dir,
    transform_resolver,
    captured_topics,
    wait_timeout_sec,
    service_timeout_sec,
    tf_timeout_sec,
    min_food_point_count,
    tcp_from_finger_base,
    feedback_cb,
    pointcloud_snapshotter,
    snapshot_captured_cb,
    food_cloud_publisher,
):
    """Run the pipeline with a persistent point-cloud snapshotter."""
    initial_sequences = pointcloud_snapshotter.mark()
    combined = combine_pointclouds.capture_combined_cloud(
        node,
        transform_resolver,
        topics=captured_topics,
        wait_timeout_sec=wait_timeout_sec,
        tf_timeout_sec=tf_timeout_sec,
        feedback_cb=feedback_cb,
        snapshotter=pointcloud_snapshotter,
        after_sequences=initial_sequences,
        snapshot_captured_cb=snapshot_captured_cb,
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
    base_from_cloud_matrix = transform_resolver.lookup_base_from_camera(
        combined.frame_id,
        stamp=combined.msg.header.stamp,
        timeout_sec=tf_timeout_sec,
    )
    food_indices = food_detector.detect_food(
        stored_cloud,
        combined.msg,
        base_from_cloud_matrix,
    )
    food_presence.require_minimum_food_points(
        food_indices,
        min_food_point_count,
    )

    # Publish before the GPD call, not after: the octomap updater resolves the
    # cloud's frame at the cloud's own stamp, and the GPD detection takes long
    # enough (minutes) that the capture stamp would have aged out of
    # move_group's tf2 buffer by the time it returns.
    _emit(feedback_cb, 'Publishing the food points for the planning octomap')
    food_cloud_publisher.publish(combined.msg, food_indices)

    _emit(feedback_cb, 'Cropping the GPD cloud around the detected food')
    cropped_msg, cropped_food_indices, cropped_camera_source = (
        gpd_request_builder.crop_cloud_around_indices(
            combined.msg,
            food_indices,
            camera_source=combined.camera_source,
            radius=DEFAULT_GPD_CLOUD_CROP_RADIUS,
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
    grasp_config_list = _call_gpd_service(node, request, service_timeout_sec)

    _emit(feedback_cb, 'Converting GPD grasps into base_link poses')
    poses = grasp_config_conversion.grasp_configs_to_poses(
        grasp_config_list,
        tcp_from_finger_base=tcp_from_finger_base,
        target_from_grasp_frame=base_from_cloud_matrix,
        target_frame=camera_transforms.BASE_LINK_FRAME,
    )
    for pose in poses:
        pose.header.stamp = combined.msg.header.stamp
    _emit(feedback_cb, 'Filtering grasps by shelf approach direction')
    ranked_poses = grasp_pose_ranking.prefer_shelf_approaches(poses)
    node.get_logger().info(
        f'Kept {len(ranked_poses)} of {len(poses)} grasps after excluding '
        'approaches outside '
        f'{grasp_pose_ranking.DEFAULT_MINIMUM_POSITIVE_X_ANGLE_DEG:g} degrees '
        'to '
        f'{grasp_pose_ranking.DEFAULT_MAXIMUM_POSITIVE_X_ANGLE_DEG:g} degrees '
        'from map +X toward the shelf'
    )
    return ranked_poses


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
