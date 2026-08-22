"""Assemble ``DetectConstrainedGrasps`` service requests for the GPD server.

Turns a captured cloud plus the food-point indices (as produced by
:func:`grasp_pose_provider.food_detector.detect_food`) into the request the
``detect_constrained_grasps`` service expects.
"""

from geometry_msgs.msg import Point
from std_msgs.msg import Int64

from gpd_ros2_msgs.msg import CloudIndexed, CloudSources
from gpd_ros2_msgs.srv import DetectConstrainedGrasps


def build_cloud_indexed(cloud_msg, indices, camera_source=None, view_points=None):
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


def build_service_request(indices, cloud_msg, camera_source=None, view_points=None):
    """Build a ``DetectConstrainedGrasps.Request`` for the given food points.

    ``camera_source`` and ``view_points`` are passed straight through to
    :func:`build_cloud_indexed`. ``grasp_params`` is left empty and
    ``params_policy`` is set to ``USE_CFG_FILE`` (0) so the server uses its own
    cfg file for the detection parameters.
    """
    request = DetectConstrainedGrasps.Request()
    request.cloud_indexed = build_cloud_indexed(
        cloud_msg, indices, camera_source=camera_source, view_points=view_points
    )
    request.params_policy = DetectConstrainedGrasps.Request.USE_CFG_FILE
    return request
