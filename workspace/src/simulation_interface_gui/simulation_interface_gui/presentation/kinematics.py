"""Pure transformation helpers for the robot's top-view representation."""

import math

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Return planar yaw from a possibly non-unit quaternion."""
    norm_squared = (
        quaternion.w * quaternion.w
        + quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    if norm_squared <= 1e-24:
        raise ValueError('Cannot calculate yaw from a zero quaternion.')

    scale = 1.0 / math.sqrt(norm_squared)
    w = quaternion.w * scale
    x = quaternion.x * scale
    y = quaternion.y * scale
    z = quaternion.z * scale
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def robot_point_to_world(
    point: Point3D,
    base_position: Point3D,
    base_yaw: float,
) -> Point3D:
    """Rotate a chassis-relative point by yaw and translate it into the world."""
    cosine = math.cos(base_yaw)
    sine = math.sin(base_yaw)
    return Point3D(
        x=base_position.x + cosine * point.x - sine * point.y,
        y=base_position.y + sine * point.x + cosine * point.y,
        z=base_position.z + point.z,
    )
