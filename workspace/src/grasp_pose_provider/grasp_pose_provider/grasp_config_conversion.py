"""Convert GPD ``GraspConfigList`` results into TCP grasp poses.

A ``GraspConfig`` uses GPD's hand frame: X is approach, Y is the closing
binormal, Z is the hand axis, and position is the base of the fingers. The
robot's ``arm_tcp`` frame instead approaches along +Z and the parallel jaws
close along +X/-X. It sits at the mid-height of the jaw plates, so the jaws
extend 40 mm beyond it. This module applies that fixed axis and position
conversion before the poses are handed to MoveIt.
"""

from geometry_msgs.msg import PoseStamped
import numpy as np


# Must stay in step with the jaw plates in robot_description's gripper.xacro
# and with hand_depth in gpd_ros2's ros_eigen_params.cfg: the plates are 80 mm
# long and arm_tcp sits at their mid-height, 40 mm short of their tips.
DEFAULT_GPD_HAND_DEPTH = 0.08
DEFAULT_FINGER_TIP_FROM_TCP = 0.04
DEFAULT_TCP_FROM_FINGER_BASE = (
    DEFAULT_GPD_HAND_DEPTH - DEFAULT_FINGER_TIP_FROM_TCP
)


def grasp_configs_to_poses(
    grasp_config_list,
    tcp_from_finger_base=DEFAULT_TCP_FROM_FINGER_BASE,
    target_from_grasp_frame=None,
    target_frame=None,
):
    """Convert a ``GraspConfigList`` into a list of TCP grasp poses.

    ``tcp_from_finger_base`` is the distance from GPD's finger-base position to
    ``arm_tcp``. In TCP coordinates, +Z is GPD approach, +X is GPD binormal
    (finger closing), and +Y is GPD axis. This matches the gripper URDF, whose
    left and right jaws travel along the TCP X axis.

    If ``target_from_grasp_frame`` and ``target_frame`` are supplied, the
    resulting poses are transformed using the transform captured with the
    input point cloud. This anchors them before a moving camera frame changes
    while GPD is processing.
    """
    if tcp_from_finger_base < 0.0:
        raise ValueError('tcp_from_finger_base must be non-negative.')
    if (target_from_grasp_frame is None) != (target_frame is None):
        raise ValueError(
            'target_from_grasp_frame and target_frame must be supplied '
            'together.'
        )

    target_matrix = None
    if target_from_grasp_frame is not None:
        target_matrix = np.asarray(target_from_grasp_frame, dtype=np.float64)
        if target_matrix.shape != (4, 4):
            raise ValueError('target_from_grasp_frame must be a 4x4 matrix.')

    poses = []
    for grasp in grasp_config_list.grasps:
        pose = PoseStamped()
        pose.header.stamp = grasp_config_list.header.stamp
        pose.header.frame_id = (
            grasp_config_list.header.frame_id
            if target_frame is None
            else target_frame
        )

        position = np.array(
            [grasp.position.x, grasp.position.y, grasp.position.z],
            dtype=np.float64,
        ) + tcp_from_finger_base * np.array(
            [grasp.approach.x, grasp.approach.y, grasp.approach.z],
            dtype=np.float64,
        )

        rotation = _grasp_axes_to_tcp_rotation(
            grasp.approach, grasp.binormal, grasp.axis
        )
        if target_matrix is not None:
            position = (
                target_matrix[:3, :3] @ position + target_matrix[:3, 3]
            )
            rotation = target_matrix[:3, :3] @ rotation

        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])

        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rotation)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        poses.append(pose)
    return poses


def _grasp_axes_to_tcp_quaternion(approach, binormal, axis):
    """Return the TCP quaternion corresponding to GPD's hand axes.

    The TCP rotation columns are ``[binormal, axis, approach]``: TCP +X is the
    jaw closing direction and TCP +Z is the direction in which the gripper
    advances onto the object.
    """
    return _rotation_matrix_to_quaternion(
        _grasp_axes_to_tcp_rotation(approach, binormal, axis)
    )


def _grasp_axes_to_tcp_rotation(approach, binormal, axis):
    """Return the TCP rotation matrix corresponding to GPD's hand axes."""
    return np.array(
        [
            [binormal.x, axis.x, approach.x],
            [binormal.y, axis.y, approach.y],
            [binormal.z, axis.z, approach.z],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to a quaternion ``(x, y, z, w)``."""
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)
