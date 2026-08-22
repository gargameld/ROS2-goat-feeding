"""Convert GPD ``GraspConfigList`` results into TCP grasp poses.

A ``GraspConfig`` uses GPD's hand frame: X is approach, Y is the closing
binormal, Z is the hand axis, and position is the base of the fingers. The
robot's ``arm_tcp`` frame instead approaches along +Z and sits at the finger
tips. This module applies that fixed axis and position conversion before the
poses are handed to MoveIt.
"""

import numpy as np
from geometry_msgs.msg import PoseStamped


DEFAULT_HAND_DEPTH = 0.037


def grasp_configs_to_poses(
    grasp_config_list,
    hand_depth=DEFAULT_HAND_DEPTH,
):
    """Convert a ``GraspConfigList`` into a list of TCP grasp poses.

    ``hand_depth`` is the distance from GPD's finger-base position to
    ``arm_tcp`` at the fingertips. In TCP coordinates, +Z is GPD approach, +Y
    is GPD binormal (finger closing), and +X is negative GPD axis. The sign on
    X keeps the resulting rotation right-handed.
    """
    if hand_depth < 0.0:
        raise ValueError('hand_depth must be non-negative.')

    poses = []
    for grasp in grasp_config_list.grasps:
        pose = PoseStamped()
        pose.header = grasp_config_list.header

        pose.pose.position.x = (
            grasp.position.x + hand_depth * grasp.approach.x
        )
        pose.pose.position.y = (
            grasp.position.y + hand_depth * grasp.approach.y
        )
        pose.pose.position.z = (
            grasp.position.z + hand_depth * grasp.approach.z
        )

        qx, qy, qz, qw = _grasp_axes_to_tcp_quaternion(
            grasp.approach, grasp.binormal, grasp.axis
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        poses.append(pose)
    return poses


def _grasp_axes_to_tcp_quaternion(approach, binormal, axis):
    """Return the TCP quaternion corresponding to GPD's hand axes.

    The TCP rotation columns are ``[-axis, binormal, approach]`` so its +Z axis
    is the direction in which the gripper advances onto the object.
    """
    rotation = np.array(
        [
            [-axis.x, binormal.x, approach.x],
            [-axis.y, binormal.y, approach.y],
            [-axis.z, binormal.z, approach.z],
        ],
        dtype=np.float64,
    )
    return _rotation_matrix_to_quaternion(rotation)


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
