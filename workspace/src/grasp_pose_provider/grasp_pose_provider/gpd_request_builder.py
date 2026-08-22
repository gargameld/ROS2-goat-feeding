"""Assemble ``DetectConstrainedGrasps`` service requests for the GPD server.

Turns a captured cloud plus the food-point indices (as produced by
:func:`grasp_pose_provider.food_detector.detect_food`) into the request the
``detect_constrained_grasps`` service expects.
"""

from geometry_msgs.msg import Point
from gpd_ros2_msgs.msg import CloudIndexed, CloudSources
from gpd_ros2_msgs.srv import DetectConstrainedGrasps
import numpy as np
import open3d as o3d
from std_msgs.msg import Int64

from grasp_pose_provider import pointcloud_conversion


DEFAULT_CLOUD_CROP_RADIUS = 0.10


def crop_cloud_around_indices(
    cloud_msg,
    indices,
    camera_source=None,
    radius=DEFAULT_CLOUD_CROP_RADIUS,
):
    """Crop a dense cloud to points within ``radius`` of indexed food points.

    Returns ``(cropped_msg, cropped_indices, cropped_camera_source)``. The food
    indices are remapped into the cropped cloud, and per-point camera-source
    entries are cropped by the same mask. ``view_points`` need no adjustment
    because camera positions remain expressed in the unchanged cloud frame.

    The grasp-pose pipeline creates a finite, dense merged cloud. Reject a
    cloud containing non-finite points here rather than silently corrupting
    index correspondence.
    """
    if radius < 0.0:
        raise ValueError('radius must be non-negative.')

    num_points = cloud_msg.width * cloud_msg.height
    points = pointcloud_conversion.finite_points(cloud_msg)
    if points.shape[0] != num_points:
        raise ValueError('GPD cloud cropping requires a finite, dense cloud.')

    indices = np.asarray(indices, dtype=np.int64).reshape(-1)
    if np.any(indices < 0) or np.any(indices >= num_points):
        raise IndexError('food indices must refer to points in cloud_msg.')

    if camera_source is not None:
        camera_source = np.asarray(camera_source, dtype=np.int64).reshape(-1)
        if camera_source.shape[0] != num_points:
            raise ValueError(
                'camera_source must contain one entry per cloud point.'
            )

    if indices.size == 0:
        keep_indices = np.empty(0, dtype=np.int64)
    else:
        scene_cloud = o3d.geometry.PointCloud()
        scene_cloud.points = o3d.utility.Vector3dVector(points)
        food_cloud = scene_cloud.select_by_index(indices.tolist())
        distances = np.asarray(
            scene_cloud.compute_point_cloud_distance(food_cloud)
        )
        keep_indices = np.flatnonzero(distances <= radius)

    old_to_new = np.full(num_points, -1, dtype=np.int64)
    old_to_new[keep_indices] = np.arange(keep_indices.size, dtype=np.int64)
    cropped_indices = old_to_new[indices]
    if np.any(cropped_indices < 0):
        raise RuntimeError('Cloud crop unexpectedly removed a food point.')

    cropped_msg = pointcloud_conversion.numpy_to_ros(
        points[keep_indices],
        frame_id=cloud_msg.header.frame_id,
        stamp=cloud_msg.header.stamp,
    )
    cropped_camera_source = (
        None if camera_source is None else camera_source[keep_indices]
    )
    return cropped_msg, cropped_indices, cropped_camera_source


def build_cloud_indexed(
    cloud_msg,
    indices,
    camera_source=None,
    view_points=None,
):
    """Build a ``CloudIndexed`` from a captured cloud and sample indices.

    ``cloud_msg`` is the captured ``sensor_msgs/msg/PointCloud2`` and
    ``indices`` are indices into its points (the food points). The GPD server
    matches ``camera_source`` and the indices against the *full* cloud, so
    ``camera_source`` carries one entry per point in ``cloud_msg``.

    ``camera_source`` and ``view_points`` describe a cloud merged from several
    cameras (see :mod:`grasp_pose_provider.combine_pointclouds`):
    ``camera_source`` gives, per point, which camera saw it, and
    ``view_points`` gives each camera's position in the cloud's frame. GPD uses
    them to point surface normals back at the camera that observed each point,
    which only comes out right when every camera is listed. Omitting both falls
    back to a single camera sitting at the frame origin.
    """
    num_points = cloud_msg.width * cloud_msg.height

    sources = CloudSources()
    sources.cloud = cloud_msg
    if camera_source is None:
        # Single camera: every point was acquired by camera 0.
        sources.camera_source = [Int64(data=0) for _ in range(num_points)]
    else:
        sources.camera_source = [Int64(data=int(s)) for s in camera_source]

    if view_points is None:
        # One viewpoint at the origin: the cloud is already expressed relative
        # to the camera frame, so the camera sits at the frame origin.
        sources.view_points = [Point()]
    else:
        sources.view_points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            for p in view_points
        ]

    if len(sources.camera_source) != num_points:
        raise ValueError(
            f'camera_source has {len(sources.camera_source)} entries but the '
            f'cloud has {num_points} points.'
        )

    cloud_indexed = CloudIndexed()
    cloud_indexed.cloud_sources = sources
    cloud_indexed.indices = [Int64(data=int(i)) for i in indices]
    return cloud_indexed


def build_service_request(
    indices,
    cloud_msg,
    camera_source=None,
    view_points=None,
):
    """Build a ``DetectConstrainedGrasps.Request`` for the given food points.

    ``camera_source`` and ``view_points`` are passed straight through to
    :func:`build_cloud_indexed`. ``grasp_params`` is left empty and
    ``params_policy`` is set to ``USE_CFG_FILE`` (0) so the server uses its own
    cfg file for the detection parameters.
    """
    request = DetectConstrainedGrasps.Request()
    request.cloud_indexed = build_cloud_indexed(
        cloud_msg,
        indices,
        camera_source=camera_source,
        view_points=view_points,
    )
    request.params_policy = DetectConstrainedGrasps.Request.USE_CFG_FILE
    return request
