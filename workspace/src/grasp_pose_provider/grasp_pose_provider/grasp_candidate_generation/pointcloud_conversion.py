"""Conversions between stored / ROS 2 point clouds and Open3D point clouds.

Both public conversion functions temporarily persist their input to disk so
the intermediate clouds can be inspected offline while debugging the ICP
registration. The dumping is best-effort and never propagates failures into
the conversion result. Remove ``_debug_dump`` (and its call sites) once the
registration pipeline is trusted.

``read_stored_pointcloud_msg`` reads a ``sensor_msgs/msg/PointCloud2`` dumped
to YAML by ``ros2 topic echo`` (``.yaml`` / ``.yml``) back into a message --
header included, so the caller knows which camera frame the dump was recorded
in. :mod:`grasp_pose_provider.grasp_candidate_generation.stored_model` uses
that to merge the three stored per-camera dumps into a single model cloud.
"""

import array
import os
from datetime import datetime

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


# Directory the debug dumps are written to. Overridable via an environment
# variable so it can be pointed at a shared volume without code changes.
DEBUG_DUMP_DIR = os.environ.get(
    'GRASP_POSE_PROVIDER_DEBUG_DIR',
    os.path.join(os.path.expanduser('~'), 'grasp_pose_provider_debug'),
)


def _debug_dump(cloud, tag):
    """Best-effort write of ``cloud`` to a timestamped ``.pcd`` for debugging.

    Returns the path written to (or ``None`` if the dump failed). Any error is
    swallowed on purpose: debug tooling must never break the conversion.
    """
    try:
        os.makedirs(DEBUG_DUMP_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = os.path.join(DEBUG_DUMP_DIR, f'{tag}_{stamp}.pcd')
        o3d.io.write_point_cloud(path, cloud)
        return path
    except Exception:  # noqa: BLE001 - debugging aid must be non-fatal
        return None


def _pointcloud2_to_open3d(msg, return_indices=False):
    """Core PointCloud2 -> Open3D conversion (xyz only, NaNs dropped).

    Does not perform any debug dumping; callers own that so they can tag the
    dump with the right origin.

    Open3D cannot hold NaN points, so points containing a non-finite coordinate
    are dropped. That renumbers the surviving points, which breaks any caller
    that needs to map an Open3D point back to its slot in the original message
    (the GPD server indexes into the *full* cloud, NaNs included). When
    ``return_indices`` is set, the original row index of every surviving point
    is returned alongside the cloud so callers can translate back.
    """
    xyz = point_cloud2.read_points_numpy(
        msg, field_names=('x', 'y', 'z'), skip_nans=False
    )
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)

    # Row indices into the original message of the points Open3D can keep.
    valid_indices = np.nonzero(np.isfinite(xyz).all(axis=1))[0]
    xyz = xyz[valid_indices]

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    if return_indices:
        return cloud, valid_indices
    return cloud


def _pointcloud2_from_yaml(path):
    """Reconstruct a ``sensor_msgs/msg/PointCloud2`` from a ``ros2 topic echo``
    YAML dump (as produced by ``ros2 topic echo --full-length <topic>``)."""
    import yaml
    from sensor_msgs.msg import PointField

    # ``ros2 topic echo`` appends a trailing '---' document separator (and may
    # contain several messages if captured without --once), so parse the
    # stream and take the first PointCloud2-shaped document.
    record = None
    with open(path, 'r') as handle:
        for document in yaml.safe_load_all(handle):
            if isinstance(document, dict) and 'data' in document \
                    and 'fields' in document:
                record = document
                break

    if record is None:
        raise ValueError(
            f"'{path}' is not a recognizable PointCloud2 YAML dump."
        )

    msg = PointCloud2()
    header = record.get('header') or {}
    msg.header.frame_id = str(header.get('frame_id', ''))
    stamp = header.get('stamp') or {}
    msg.header.stamp.sec = int(stamp.get('sec', 0))
    msg.header.stamp.nanosec = int(stamp.get('nanosec', 0))

    msg.height = int(record['height'])
    msg.width = int(record['width'])
    msg.fields = [
        PointField(
            name=str(field['name']),
            offset=int(field['offset']),
            datatype=int(field['datatype']),
            count=int(field['count']),
        )
        for field in record['fields']
    ]
    msg.is_bigendian = bool(record['is_bigendian'])
    msg.point_step = int(record['point_step'])
    msg.row_step = int(record['row_step'])
    msg.is_dense = bool(record.get('is_dense', False))
    # 'data' is a flat list of uint8 values; feed it as a byte buffer.
    msg.data = array.array('B', record['data'])
    return msg


def read_stored_pointcloud_msg(path):
    """Read a stored ``ros2 topic echo`` YAML dump back into a ``PointCloud2``.

    The message is returned whole rather than converted straight to Open3D so
    the caller can read ``header.frame_id`` and place the cloud relative to the
    other cameras' dumps.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext not in ('.yaml', '.yml'):
        raise ValueError(
            f"'{path}' is not a YAML point cloud dump; a stored cloud has to "
            'carry the header naming the camera frame it was recorded in.'
        )
    return _pointcloud2_from_yaml(path)


def finite_points(msg):
    """Return the ``(N, 3)`` finite XYZ points of a ``PointCloud2``.

    The cameras mark rays that hit nothing as NaN, and those points carry no
    geometry to merge, so they are dropped here. Doing so renumbers the
    surviving points; callers that have to map a point back to its slot in the
    original message want :func:`ros_to_open3d` with ``return_indices``.
    """
    xyz = point_cloud2.read_points_numpy(
        msg, field_names=('x', 'y', 'z'), skip_nans=False
    )
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def ros_to_open3d(msg, return_indices=False):
    """Convert a ``sensor_msgs/msg/PointCloud2`` into an Open3D point cloud.

    Only the ``x``/``y``/``z`` fields are used; NaN points are dropped. The
    resulting cloud is dumped to :data:`DEBUG_DUMP_DIR` for inspection.

    When ``return_indices`` is set, also returns an ``int`` array giving, for
    each surviving Open3D point, its row index in ``msg`` (see
    :func:`_pointcloud2_to_open3d`).
    """
    if not isinstance(msg, PointCloud2):
        raise TypeError(
            f'ros_to_open3d expected sensor_msgs/PointCloud2, got {type(msg)!r}'
        )

    if return_indices:
        cloud, indices = _pointcloud2_to_open3d(msg, return_indices=True)
        _debug_dump(cloud, 'captured')
        return cloud, indices

    cloud = _pointcloud2_to_open3d(msg)
    _debug_dump(cloud, 'captured')
    return cloud


def numpy_to_ros(points, frame_id='', stamp=None):
    """Convert an ``(N, 3)`` array of points into a ``PointCloud2``.

    Produces an unorganized (``height == 1``) XYZ float32 cloud, marked dense:
    callers are expected to have dropped any non-finite point already.
    ``frame_id`` and ``stamp`` populate the header, with ``stamp`` left at its
    default when ``None``.
    """
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    msg = PointCloud2()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg
