"""Transforms between camera frames and ``base_link``.

The three on-board cameras each publish their cloud in their own optical frame
(``left_camera_frame``, ``left_back_camera_frame``,
``left_front_camera_frame``). Those frames are part of the robot description,
so ``robot_state_publisher`` broadcasts them on ``/tf_static``; this module
listens to that broadcast and hands back the rigid transform between any two
frames as a 4x4 homogeneous matrix ready to be applied to an Nx3 point array.
Convenience methods expose camera-to-``base_link`` transforms for geometry
checks that need the robot's gravity-aligned Z axis.

:class:`CameraTransformResolver` owns the tf2 buffer and listener. Construct
one per node and keep it alive: the buffer only answers lookups for transforms
it has already received, so a short-lived resolver would spend its whole life
waiting for the first ``/tf_static`` message. It is handed the node's
:class:`grasp_pose_provider.node_parameters.GraspPoseProviderParameters`, which
is where the buffer's cache time, the lookup timeout and ``base_link``'s name
come from. Used by
:mod:`grasp_pose_provider.grasp_candidate_generation.combine_pointclouds` to
patch the three clouds into a single one.
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer


def transform_to_matrix(transform):
    """Convert a ``geometry_msgs/msg/Transform`` into a 4x4 homogeneous matrix.

    ``transform`` maps points from the source frame into the target frame, and
    so does the returned matrix.
    """
    rotation = transform.rotation
    translation = transform.translation

    matrix = np.identity(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_rotation_matrix(
        rotation.x, rotation.y, rotation.z, rotation.w
    )
    matrix[:3, 3] = (translation.x, translation.y, translation.z)
    return matrix


def quaternion_to_rotation_matrix(x, y, z, w):
    """Convert a unit quaternion into a 3x3 rotation matrix.

    Written out here rather than pulled from ``tf_transformations`` so the
    package keeps to the ROS 2 core dependencies it already declares.
    """
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError('Cannot build a rotation from a zero quaternion.')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def apply_transform(matrix, points):
    """Apply a 4x4 homogeneous ``matrix`` to an ``(N, 3)`` array of points."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        return points
    return points @ matrix[:3, :3].T + matrix[:3, 3]


class CameraTransformResolver:
    """Looks up camera and base transforms from robot_state_publisher.

    ``node`` is only used to create the tf2 subscriptions. ``TransformListener``
    puts them on a reentrant callback group, so a multi-threaded executor keeps
    filling the buffer even while another of the node's callbacks is blocked
    inside :meth:`lookup_matrix`. ``spin_thread`` is left off on purpose: it
    would hand ``node`` to a second, private executor, and a node can only
    belong to one.
    """

    def __init__(
        self,
        node,
        parameters,
        spin_thread=False,
    ):
        self._node = node
        self._parameters = parameters
        self._buffer = Buffer(
            cache_time=Duration(seconds=parameters.tf_cache_time_sec)
        )
        self._listener = TransformListener(
            self._buffer, node, spin_thread=spin_thread
        )

    def lookup_matrix(
        self,
        target_frame,
        source_frame,
        stamp=None,
    ):
        """Return the 4x4 matrix mapping ``source_frame`` into ``target_frame``.

        ``stamp`` is a ``builtin_interfaces/msg/Time`` (typically a cloud's
        ``header.stamp``); ``None`` -- the default -- asks for the latest
        available transform, which is what the static camera frames always
        resolve to anyway. Raises ``tf2_ros.TransformException`` (or one of its
        subclasses) if the transform is still unavailable after the node's
        ``tf_timeout_sec``.
        """
        if target_frame == source_frame:
            return np.identity(4, dtype=np.float64)

        time = (
            rclpy.time.Time()
            if stamp is None
            else rclpy.time.Time.from_msg(stamp)
        )
        transform = self._buffer.lookup_transform(
            target_frame,
            source_frame,
            time,
            timeout=Duration(seconds=self._parameters.tf_timeout_sec),
        )
        return transform_to_matrix(transform.transform)

    def lookup_base_from_camera(self, camera_frame, stamp=None):
        """Map points from ``camera_frame`` into the node's base frame."""
        return self.lookup_matrix(
            self._parameters.base_frame,
            camera_frame,
            stamp=stamp,
        )

    def destroy(self):
        """Drop the listener's tf subscriptions.

        Only ever called while the node is shutting down, so a failure here is
        swallowed rather than allowed to mask the real shutdown path.
        """
        try:
            self._listener.unregister()
        except Exception:  # noqa: BLE001 - shutdown cleanup must not raise
            pass
