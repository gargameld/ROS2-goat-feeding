"""Food detection on a plate from an already-captured camera cloud.

Given one captured ``sensor_msgs/msg/PointCloud2`` (the caller is responsible
for grabbing it off the camera topic), this module isolates the food on the
plate and reports which points of the captured cloud belong to it.

The pipeline: ICP-register the stored model of the empty plate onto the
captured scene, move the model into the scene with the resulting transform,
then subtract it from the captured cloud. Whatever the plate model does not
explain is the food.

The stored model arrives already loaded and merged, as an Open3D cloud in the
captured cloud's frame; assembling it from the three per-camera dumps is
:mod:`grasp_pose_provider.stored_model`'s job. Both the ICP registration and
the cloud subtraction run entirely in Open3D form; the geometry helper for
subtraction lives in :mod:`grasp_pose_provider.subtract_pointclouds`.
"""

import numpy as np
import open3d as o3d

from grasp_pose_provider import (
    camera_transforms,
    debug_dump,
    pointcloud_conversion,
    subtract_pointclouds,
)


# ICP max correspondence distance (metres). Pairs farther apart are ignored.
DEFAULT_MAX_CORRESPONDENCE_DISTANCE = 0.05
# DBSCAN parameters for separating disconnected food/subtraction regions.
DEFAULT_CLUSTER_EPS = 0.02
DEFAULT_CLUSTER_MIN_POINTS = 10
# Shelves and walls are approximately constant along at least one base_link
# axis, while large subtraction artifacts can be much bigger than food. A food
# cluster must fit between these dimensions along every base_link axis. The
# middle 90% is used so a few depth-camera outliers do not decide the result.
DEFAULT_MIN_CLUSTER_AXIS_SPAN = 0.01
DEFAULT_MAX_CLUSTER_AXIS_SPAN = 0.10
CLUSTER_SPAN_PERCENTILES = (5.0, 95.0)
CLUSTER_SPAN_ABSOLUTE_TOLERANCE = 1e-12


def detect_food(
    stored_cloud,
    captured_cloud_msg,
    base_from_cloud_matrix,
    max_correspondence_distance=DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
    distance_threshold=subtract_pointclouds.DEFAULT_DISTANCE_THRESHOLD,
    cluster_eps=DEFAULT_CLUSTER_EPS,
    cluster_min_points=DEFAULT_CLUSTER_MIN_POINTS,
    min_cluster_axis_span=DEFAULT_MIN_CLUSTER_AXIS_SPAN,
    max_cluster_axis_span=DEFAULT_MAX_CLUSTER_AXIS_SPAN,
    debug_stage=None,
):
    """Return the indices of the food points within ``captured_cloud_msg``.

    ``stored_cloud`` is the empty-plate model as an Open3D point cloud, in the
    same frame as ``captured_cloud_msg`` -- see
    :func:`grasp_pose_provider.stored_model.load_stored_model`.
    ``captured_cloud_msg`` is the merged ``sensor_msgs/msg/PointCloud2``
    captured from the cameras. The returned value is an ``int64`` array of
    indices into that message's point ordering (NaN points included)
    identifying the largest spatial cluster with meaningful variation along
    all three ``base_link`` axes, without exceeding 10 cm along any axis -- the
    same indexing the GPD server expects in a ``CloudIndexed``. Candidate
    clusters that are roughly constant in X, Y, or Z are treated as wall or
    shelf surfaces, while oversized clusters are treated as non-food objects.
    The largest remaining cluster is chosen.

    ``base_from_cloud_matrix`` maps points from ``captured_cloud_msg``'s frame
    into ``base_link``. It should come from
    :meth:`CameraTransformResolver.lookup_base_from_camera`.
    """
    # ``original_indices`` maps each surviving Open3D point back to its row in
    # the original message, since converting to Open3D drops NaN points.
    captured_cloud, original_indices = pointcloud_conversion.ros_to_open3d(
        captured_cloud_msg, return_indices=True
    )

    # ICP registration: line the stored empty-plate model up with the scene.
    result = o3d.pipelines.registration.registration_icp(
        source=stored_cloud,
        target=captured_cloud,
        max_correspondence_distance=max_correspondence_distance,
        init=np.identity(4),
        estimation_method=(
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        ),
    )

    # Move a copy of the stored plate model into the captured scene, then keep
    # only the captured points the model does not explain -- the food.
    registered_model = o3d.geometry.PointCloud(stored_cloud)
    registered_model.transform(result.transformation)
    food_indices = subtract_pointclouds.subtract_indices(
        registered_model,
        captured_cloud,
        distance_threshold=distance_threshold,
    )

    # Cluster the subtraction result. DBSCAN label -1 represents noise and is
    # never sent to GPD. Reject planar and oversized clusters in base_link
    # before selecting the remaining cluster containing most points.
    food_cloud = captured_cloud.select_by_index(food_indices.tolist())
    if len(food_cloud.points) > 0:
        cluster_labels = np.asarray(
            food_cloud.cluster_dbscan(
                eps=cluster_eps,
                min_points=cluster_min_points,
                print_progress=False,
            ),
            dtype=np.int64,
        )
        selected_mask = _largest_non_planar_cluster_mask(
            np.asarray(food_cloud.points),
            cluster_labels,
            base_from_cloud_matrix,
            min_axis_span=min_cluster_axis_span,
            max_axis_span=max_cluster_axis_span,
        )
        food_indices = food_indices[selected_mask]

    # Intermediate debugging aid: dump the transform, the stored, captured and
    # food clouds, and the largest-cluster indices (indices here are into
    # ``captured_cloud``). Remove once detection is trusted.
    debug_dump.dump_detection(
        stored_cloud,
        captured_cloud,
        result.transformation,
        food_indices,
        stage=debug_stage,
    )

    # Translate Open3D indices back to indices into the original message.
    return original_indices[food_indices].astype(np.int64)


def _largest_non_planar_cluster_mask(
    points,
    cluster_labels,
    base_from_cloud_matrix,
    min_axis_span=DEFAULT_MIN_CLUSTER_AXIS_SPAN,
    max_axis_span=DEFAULT_MAX_CLUSTER_AXIS_SPAN,
):
    """Select the largest cluster whose ``base_link`` dimensions are valid."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    cluster_labels = np.asarray(cluster_labels, dtype=np.int64)
    if cluster_labels.shape != (points.shape[0],):
        raise ValueError('cluster_labels must contain one label per point.')
    if min_axis_span < 0.0:
        raise ValueError('min_axis_span must be non-negative.')
    if max_axis_span < min_axis_span:
        raise ValueError('max_axis_span must not be smaller than min_axis_span.')

    base_points = camera_transforms.apply_transform(
        np.asarray(base_from_cloud_matrix, dtype=np.float64), points
    )
    selected_label = None
    selected_size = -1
    for label in np.unique(cluster_labels[cluster_labels >= 0]):
        cluster_mask = cluster_labels == label
        low, high = np.percentile(
            base_points[cluster_mask],
            CLUSTER_SPAN_PERCENTILES,
            axis=0,
        )
        axis_spans = high - low
        if np.any(axis_spans <= min_axis_span):
            continue
        if np.any(axis_spans > max_axis_span + CLUSTER_SPAN_ABSOLUTE_TOLERANCE):
            continue
        cluster_size = int(np.count_nonzero(cluster_mask))
        if cluster_size > selected_size:
            selected_label = label
            selected_size = cluster_size

    if selected_label is None:
        return np.zeros(points.shape[0], dtype=bool)
    return cluster_labels == selected_label
