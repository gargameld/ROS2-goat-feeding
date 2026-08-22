"""Convert GPD ``GraspConfigList`` results into TCP grasp poses.

A ``GraspConfig`` describes a grasp by a hand position and three orthonormal
axes ``[approach, binormal, axis]`` that form the columns of the grasp's
rotation matrix. This module turns those into ROS ``PoseStamped`` messages the
rest of the pipeline (and, later, MoveIt) can consume.
"""

import numpy as np
from geometry_msgs.msg import PoseStamped


def grasp_configs_to_poses(grasp_config_list):
    """Convert a ``GraspConfigList`` into a list of TCP grasp poses.

    Each ``GraspConfig`` carries the hand position and an orientation given as
    three axes ``[approach, binormal, axis]``, which form the columns of the
    rotation matrix. Those become the ``PoseStamped`` position and orientation
    quaternion, stamped in the list's frame.
    """
    poses = []
    for grasp in grasp_config_list.grasps:
        pose = PoseStamped()
        pose.header = grasp_config_list.header

        pose.pose.position.x = grasp.position.x
        pose.pose.position.y = grasp.position.y
        pose.pose.position.z = grasp.position.z

        qx, qy, qz, qw = _grasp_axes_to_quaternion(
            grasp.approach, grasp.binormal, grasp.axis
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        poses.append(pose)
    return poses


def _grasp_axes_to_quaternion(approach, binormal, axis):
    """Quaternion (x, y, z, w) from the grasp's three orthonormal axes.

    The axes are the columns of the rotation matrix: ``R = [approach binormal
    axis]``.
    """
    rotation = np.array(
        [
            [approach.x, binormal.x, axis.x],
            [approach.y, binormal.y, axis.y],
            [approach.z, binormal.z, axis.z],
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
