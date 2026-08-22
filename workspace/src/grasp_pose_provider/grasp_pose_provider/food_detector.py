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
    debug_dump,
    pointcloud_conversion,
    subtract_pointclouds,
)


# ICP max correspondence distance (metres). Pairs farther apart are ignored.
DEFAULT_MAX_CORRESPONDENCE_DISTANCE = 0.05


def detect_food(
    stored_cloud,
    captured_cloud_msg,
    max_correspondence_distance=DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
    distance_threshold=subtract_pointclouds.DEFAULT_DISTANCE_THRESHOLD,
):
    """Return the indices of the food points within ``captured_cloud_msg``.

    ``stored_cloud`` is the empty-plate model as an Open3D point cloud, in the
    same frame as ``captured_cloud_msg`` -- see
    :func:`grasp_pose_provider.stored_model.load_stored_model`.
    ``captured_cloud_msg`` is the merged ``sensor_msgs/msg/PointCloud2``
    captured from the cameras. The returned value is an ``int64`` array of
    indices into that message's point ordering (NaN points included)
    identifying the points that belong to the food -- the same indexing the GPD
    server expects in a ``CloudIndexed``.
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

    # Intermediate debugging aid: dump the transform, the stored, captured and
    # food clouds, and the food indices (indices here are into
    # ``captured_cloud``). Remove once detection is trusted.
    debug_dump.dump_detection(
        stored_cloud, captured_cloud, result.transformation, food_indices
    )

    # Translate Open3D indices back to indices into the original message.
    return original_indices[food_indices].astype(np.int64)
