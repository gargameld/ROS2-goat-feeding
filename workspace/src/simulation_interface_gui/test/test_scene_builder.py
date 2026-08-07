"""Tests for immutable top-view scene construction."""

from dataclasses import FrozenInstanceError
import math

import pytest

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import SimulationSnapshot
from simulation_interface_gui.presentation import SceneBuilder


def test_builder_rotates_and_translates_base():
    """The base outline and forward marker follow the snapshot pose."""
    half_sqrt = math.sqrt(0.5)
    snapshot = SimulationSnapshot(
        base_position=Point3D(2.0, 3.0, 0.26),
        base_orientation=Quaternion(half_sqrt, 0.0, 0.0, half_sqrt),
        arm_joint_positions=(0.0,) * 6,
    )

    scene = SceneBuilder().build(snapshot)

    first_corner = scene.base_polygon.points[0]
    assert first_corner.x == pytest.approx(2.5)
    assert first_corner.y == pytest.approx(2.6)
    assert scene.orientation_marker.end.x == pytest.approx(1.35)
    assert scene.orientation_marker.end.y == pytest.approx(3.0)


def test_builder_produces_arm_and_static_world_geometry():
    """A scene contains arm links, joint markers, walls, and four shelves."""
    snapshot = SimulationSnapshot(
        base_position=Point3D(0.0, 0.0, 0.26),
        base_orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
        arm_joint_positions=(0.0,) * 6,
    )

    scene = SceneBuilder().build(snapshot)

    assert len(scene.arm_segments) == 6
    assert len(scene.joint_markers) == 6
    assert len(scene.world_boundaries) == 24
    assert len(scene.obstacle_polygons) == 4


def test_scene_is_immutable():
    """Completed scenes cannot be mutated by the renderer."""
    snapshot = SimulationSnapshot(
        base_position=Point3D(0.0, 0.0, 0.26),
        base_orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
        arm_joint_positions=(0.0,) * 6,
    )
    scene = SceneBuilder().build(snapshot)

    with pytest.raises(FrozenInstanceError):
        scene.joint_markers = ()
