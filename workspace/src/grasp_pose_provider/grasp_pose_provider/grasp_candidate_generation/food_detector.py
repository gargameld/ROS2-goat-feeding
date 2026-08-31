"""Food detection on a plate from an already-captured camera cloud.

Given one captured ``sensor_msgs/msg/PointCloud2`` (the caller is responsible
for grabbing it off the camera topic), this module isolates the food on the
plate and reports which points of the captured cloud belong to it.

The pipeline: ICP-register the stored model of the empty plate onto the
captured scene, move the model into the scene with the resulting transform,
then subtract it from the captured cloud. Whatever the plate model does not
explain is the food.

Every value that tunes that pipeline -- the ICP correspondence distance, the
subtraction threshold, the clustering settings and the accepted cluster
dimensions -- is a node parameter, so none of them is hard coded here:
:func:`detect_food` is handed the node's
:class:`grasp_pose_provider.node_parameters.GraspPoseProviderParameters` and
reads them off it.

The stored model arrives already loaded and merged, as an Open3D cloud in the
captured cloud's frame; assembling it from the three per-camera dumps is
:mod:`grasp_pose_provider.grasp_candidate_generation.stored_model`'s job. Both
the ICP registration and the cloud subtraction run entirely in Open3D form; the
geometry helper for subtraction lives in
:mod:`grasp_pose_provider.grasp_candidate_generation.subtract_pointclouds`.

Judging whether what came back counts as food at all is part of detection, so
it lives here too: :func:`require_minimum_food_points` raises
:class:`NoFoodDetectedError` when the segmentation kept fewer points than
``min_food_point_count`` allows, which is how an empty plate is reported to
the caller.
"""

import numpy as np
import open3d as o3d

from grasp_pose_provider.grasp_candidate_generation import (
    camera_transforms,
    debug_dump,
    pointcloud_conversion,
    subtract_pointclouds,
)


# Slack allowed when comparing a cluster's span against the maximum, so that a
# cluster measuring exactly the limit is not rejected by floating point noise.
CLUSTER_SPAN_ABSOLUTE_TOLERANCE = 1e-12


class NoFoodDetectedError(RuntimeError):
    """Raised when segmentation finds too few food points for grasping."""

    def __init__(self, point_count, minimum_point_count):
        self.point_count = point_count
        self.minimum_point_count = minimum_point_count
        super().__init__(
            f'found {point_count} food points; at least '
            f'{minimum_point_count} are required'
        )


def detect_food(
    parameters,
    stored_cloud,
    captured_cloud_msg,
    base_from_cloud_matrix,
    debug_stage=None,
):
    """Return the indices of the food points within ``captured_cloud_msg``.

    ``stored_cloud`` is the empty-plate model as an Open3D point cloud, in the
    same frame as ``captured_cloud_msg`` -- see
    :func:`grasp_pose_provider.grasp_candidate_generation.stored_model.load_stored_model`.
    ``captured_cloud_msg`` is the merged ``sensor_msgs/msg/PointCloud2``
    captured from the cameras. The returned value is an ``int64`` array of
    indices into that message's point ordering (NaN points included)
    identifying the largest spatial cluster whose extent along every
    ``base_link`` axis falls between ``min_food_cluster_axis_span`` and
    ``max_food_cluster_axis_span`` -- the same indexing the GPD server expects
    in a ``CloudIndexed``. Candidate clusters that are roughly constant in X,
    Y, or Z are treated as wall or shelf surfaces, while oversized clusters are
    treated as non-food objects. The largest remaining cluster is chosen.

    ``base_from_cloud_matrix`` maps points from ``captured_cloud_msg``'s frame
    into ``base_link``. It should come from
    :meth:`CameraTransformResolver.lookup_base_from_camera`.

    ``parameters`` is the node's
    :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`.
    Every value tuning the detection is read off it:
    ``icp_max_correspondence_distance`` is the ICP correspondence distance in
    metres, so pairs farther apart than it are ignored during registration;
    ``food_subtraction_distance_threshold`` is how close a captured point must
    be to the registered model to count as explained by it; ``food_cluster_eps``
    and ``food_cluster_min_points`` are the DBSCAN settings used to separate
    the disconnected regions the subtraction leaves behind; and
    ``food_cluster_span_percentiles`` are the two percentiles a cluster's
    extent is measured between, so a few depth-camera outliers do not decide
    the result.
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
        max_correspondence_distance=(
            parameters.icp_max_correspondence_distance
        ),
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
        distance_threshold=parameters.food_subtraction_distance_threshold,
    )

    # Cluster the subtraction result. DBSCAN label -1 represents noise and is
    # never sent to GPD. Reject planar and oversized clusters in base_link
    # before selecting the remaining cluster containing most points.
    food_cloud = captured_cloud.select_by_index(food_indices.tolist())
    if len(food_cloud.points) > 0:
        cluster_labels = np.asarray(
            food_cloud.cluster_dbscan(
                eps=parameters.food_cluster_eps,
                min_points=parameters.food_cluster_min_points,
                print_progress=False,
            ),
            dtype=np.int64,
        )
        selected_mask = _largest_non_planar_cluster_mask(
            np.asarray(food_cloud.points),
            cluster_labels,
            base_from_cloud_matrix,
            min_axis_span=parameters.min_food_cluster_axis_span,
            max_axis_span=parameters.max_food_cluster_axis_span,
            span_percentiles=parameters.food_cluster_span_percentiles,
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


def require_minimum_food_points(parameters, food_indices):
    """Raise :class:`NoFoodDetectedError` for an empty/tiny segmentation.

    ``food_indices`` is what :func:`detect_food` returned. The threshold is the
    node's ``min_food_point_count`` parameter, read off the
    :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`
    passed in.
    """
    minimum_point_count = parameters.min_food_point_count
    if minimum_point_count < 1:
        raise ValueError('min_food_point_count must be at least 1.')

    point_count = len(food_indices)
    if point_count < minimum_point_count:
        raise NoFoodDetectedError(point_count, minimum_point_count)


def _largest_non_planar_cluster_mask(
    points,
    cluster_labels,
    base_from_cloud_matrix,
    min_axis_span,
    max_axis_span,
    span_percentiles,
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
            list(span_percentiles),
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
