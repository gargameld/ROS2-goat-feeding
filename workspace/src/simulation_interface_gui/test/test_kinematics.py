"""Tests for pure robot kinematics."""

import math

import pytest

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.presentation.kinematics import ArmKinematics
from simulation_interface_gui.presentation.kinematics import quaternion_to_yaw
from simulation_interface_gui.presentation.kinematics import robot_point_to_world


def test_quaternion_to_yaw_normalizes_input():
    """Yaw conversion supports non-unit MuJoCo quaternion values."""
    yaw = quaternion_to_yaw(Quaternion(2.0, 0.0, 0.0, 2.0))

    assert yaw == pytest.approx(math.pi / 2.0)


def test_robot_point_to_world_applies_planar_pose():
    """A robot-relative point is rotated and translated into world space."""
    transformed = robot_point_to_world(
        Point3D(0.0, 1.0, 0.5),
        Point3D(2.0, 3.0, 4.0),
        math.pi / 2.0,
    )

    assert transformed.x == pytest.approx(1.0)
    assert transformed.y == pytest.approx(3.0)
    assert transformed.z == pytest.approx(4.5)


def test_arm_forward_returns_six_joints_and_tool():
    """The MJCF arm chain produces one point per joint and one tool point."""
    points = ArmKinematics().forward((0.0,) * 6)

    assert len(points) == 7
    assert all(math.isfinite(value) for point in points
               for value in (point.x, point.y, point.z))


def test_arm_forward_requires_six_joint_values():
    """Incomplete arm state is rejected with a clear error."""
    with pytest.raises(ValueError, match='Expected 6 arm joints'):
        ArmKinematics().forward((0.0,) * 5)
