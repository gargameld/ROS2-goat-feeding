"""Compensate scene motion between the pre- and post-GPD snapshots."""

import numpy as np
import open3d as o3d

from grasp_pose_provider import pointcloud_conversion


DEFAULT_VOXEL_SIZE = 0.005
DEFAULT_MAX_CORRESPONDENCE_DISTANCE = 0.05
DEFAULT_MIN_POINTS = 100
DEFAULT_CENTER_PERCENTILES = (5.0, 95.0)


def estimate_new_from_old_food(
    old_cloud_msg,
    old_food_indices,
    new_cloud_msg,
    new_food_indices,
):
    """Return a translation mapping the old detected food onto the new one.

    Whole-scene point-to-point ICP cannot observe translation along the large,
    repetitive shelf planes reliably.  The food is the geometry the grasps
    are attached to, so use the change in its robust point-cloud center
    directly.  Orientations are intentionally left unchanged: this correction
    addresses the stale grasp positions without perturbing otherwise valid
    GPD orientations.
    """
    old_points = _indexed_finite_points(old_cloud_msg, old_food_indices)
    new_points = _indexed_finite_points(new_cloud_msg, new_food_indices)
    old_center = _robust_center(old_points)
    new_center = _robust_center(new_points)

    transformation = np.identity(4, dtype=np.float64)
    transformation[:3, 3] = new_center - old_center
    return transformation


def recenter_grasps_on_food_binormal(
    grasp_config_list,
    new_cloud_msg,
    new_food_indices,
    new_from_old_transform,
):
    """Center every GPD hand on the updated food along its closing axis.

    GPD chooses from a discrete set of lateral finger placements. Its
    lower-middle tie break can put every otherwise-valid hand several
    millimetres toward the same binormal side. Preserve the selected approach
    depth and orientation, but translate the hand along its own binormal so
    its jaw midplane crosses the robust center of the newly observed food.

    ``new_from_old_transform`` maps the GPD cloud frame into the post-GPD
    cloud frame. The grasp messages remain expressed in the old frame; their
    positions are adjusted there so the transform applied downstream maps
    the adjusted jaw midplanes onto the new food center.

    Returns the signed lateral adjustment applied to each grasp, in metres.
    """
    transform = np.asarray(new_from_old_transform, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError('new_from_old_transform must be a 4x4 matrix.')

    new_points = _indexed_finite_points(new_cloud_msg, new_food_indices)
    new_center = _robust_center(new_points)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    adjustments = []
    for grasp in grasp_config_list.grasps:
        old_position = np.array(
            [grasp.position.x, grasp.position.y, grasp.position.z],
            dtype=np.float64,
        )
        old_binormal = np.array(
            [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z],
            dtype=np.float64,
        )
        binormal_norm = np.linalg.norm(old_binormal)
        if not np.isfinite(binormal_norm) or binormal_norm <= 1e-12:
            raise ValueError(
                'Every grasp binormal must be finite and nonzero.'
            )
        old_binormal /= binormal_norm

        new_position = rotation @ old_position + translation
        new_binormal = rotation @ old_binormal
        new_binormal /= np.linalg.norm(new_binormal)
        adjustment = float(
            np.dot(new_center - new_position, new_binormal)
        )

        adjusted_position = old_position + adjustment * old_binormal
        grasp.position.x = float(adjusted_position[0])
        grasp.position.y = float(adjusted_position[1])
        grasp.position.z = float(adjusted_position[2])
        adjustments.append(adjustment)

    return np.asarray(adjustments, dtype=np.float64)


def _indexed_finite_points(cloud_msg, indices):
    """Return finite points selected by dense-cloud ``indices``."""
    points = pointcloud_conversion.finite_points(cloud_msg)
    indices = np.asarray(indices, dtype=np.int64).reshape(-1)
    if indices.size == 0:
        raise ValueError('Food-point indices must not be empty.')
    if np.any(indices < 0) or np.any(indices >= points.shape[0]):
        raise IndexError('Food-point index is outside the point cloud.')
    selected = points[indices]
    if not np.isfinite(selected).all():
        raise ValueError('Food points must be finite.')
    return selected


def _robust_center(points):
    """Return the midpoint of the central 90% bounds of ``points``."""
    low, high = np.percentile(
        points,
        DEFAULT_CENTER_PERCENTILES,
        axis=0,
    )
    return 0.5 * (low + high)


def estimate_new_from_old_camera(
    old_cloud_msg,
    new_cloud_msg,
    voxel_size=DEFAULT_VOXEL_SIZE,
    max_correspondence_distance=DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
):
    """Return the rigid transform mapping old-camera coordinates to new.

    GPD reports poses in the frame of ``old_cloud_msg``.  ICP uses the old
    scene as its source and the newly captured scene as its target, so its
    transformation can be applied directly to those returned positions and
    orientations before normal downstream frame conversion.
    """
    if voxel_size <= 0.0:
        raise ValueError('voxel_size must be positive.')
    if max_correspondence_distance <= 0.0:
        raise ValueError('max_correspondence_distance must be positive.')

    # Build these directly rather than using ``ros_to_open3d``: that public
    # debugging conversion writes a full PCD file and would add substantial
    # latency to every post-GPD correction.
    old_cloud = o3d.geometry.PointCloud()
    old_cloud.points = o3d.utility.Vector3dVector(
        pointcloud_conversion.finite_points(old_cloud_msg)
    )
    new_cloud = o3d.geometry.PointCloud()
    new_cloud.points = o3d.utility.Vector3dVector(
        pointcloud_conversion.finite_points(new_cloud_msg)
    )
    old_cloud = old_cloud.voxel_down_sample(voxel_size)
    new_cloud = new_cloud.voxel_down_sample(voxel_size)
    if len(old_cloud.points) < DEFAULT_MIN_POINTS:
        raise RuntimeError('The pre-GPD cloud has too few points for ICP.')
    if len(new_cloud.points) < DEFAULT_MIN_POINTS:
        raise RuntimeError('The post-GPD cloud has too few points for ICP.')

    result = o3d.pipelines.registration.registration_icp(
        source=old_cloud,
        target=new_cloud,
        max_correspondence_distance=max_correspondence_distance,
        init=np.identity(4),
        estimation_method=(
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        ),
    )
    transformation = np.asarray(result.transformation, dtype=np.float64)
    if transformation.shape != (4, 4) or not np.isfinite(transformation).all():
        raise RuntimeError('ICP returned an invalid camera-motion transform.')
    return transformation
