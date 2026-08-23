"""Tests for object-specific post-GPD motion compensation."""

import numpy as np
import pytest

from geometry_msgs.msg import Vector3
from gpd_ros2_msgs.msg import GraspConfig, GraspConfigList

from grasp_pose_provider import motion_compensation, pointcloud_conversion


def test_food_center_translation_maps_old_snapshot_to_new_snapshot():
    old_points = np.array(
        [
            [-1.0, -2.0, -3.0],
            [0.0, 0.0, 0.0],
            [2.0, 4.0, 6.0],
            [100.0, 100.0, 100.0],
        ]
    )
    displacement = np.array([0.017, -0.003, 0.002])
    new_points = old_points.copy()
    new_points[:3] += displacement

    transform = motion_compensation.estimate_new_from_old_food(
        pointcloud_conversion.numpy_to_ros(old_points),
        [0, 1, 2],
        pointcloud_conversion.numpy_to_ros(new_points),
        [0, 1, 2],
    )

    assert np.allclose(transform[:3, :3], np.identity(3))
    assert np.allclose(transform[:3, 3], displacement)


def test_food_center_translation_rejects_empty_indices():
    cloud = pointcloud_conversion.numpy_to_ros([[0.0, 0.0, 0.0]])

    with pytest.raises(ValueError, match='must not be empty'):
        motion_compensation.estimate_new_from_old_food(
            cloud,
            [],
            cloud,
            [0],
        )


def test_recenters_grasp_only_along_its_binormal_in_old_frame():
    grasp = GraspConfig()
    grasp.position.x = 0.9
    grasp.position.y = 2.0
    grasp.position.z = 3.0
    grasp.binormal = Vector3(x=1.0, y=0.0, z=0.0)
    grasps = GraspConfigList()
    grasps.grasps = [grasp]

    # The old hand maps to x=1.0 in the new frame, while the updated food's
    # robust center is x=1.2. The old-frame hand must move +0.2 along its
    # binormal; its approach-depth coordinates must remain unchanged.
    new_cloud = pointcloud_conversion.numpy_to_ros(
        [[1.1, 0.0, 0.0], [1.3, 0.0, 0.0]]
    )
    new_from_old = np.identity(4)
    new_from_old[0, 3] = 0.1

    adjustments = motion_compensation.recenter_grasps_on_food_binormal(
        grasps,
        new_cloud,
        [0, 1],
        new_from_old,
    )

    assert np.allclose(adjustments, [0.2])
    assert np.isclose(grasp.position.x, 1.1)
    assert np.isclose(grasp.position.y, 2.0)
    assert np.isclose(grasp.position.z, 3.0)


def test_binormal_recentering_is_independent_for_each_grasp_orientation():
    x_grasp = GraspConfig()
    x_grasp.binormal = Vector3(x=2.0, y=0.0, z=0.0)
    y_grasp = GraspConfig()
    y_grasp.binormal = Vector3(x=0.0, y=-1.0, z=0.0)
    grasps = GraspConfigList()
    grasps.grasps = [x_grasp, y_grasp]
    food = pointcloud_conversion.numpy_to_ros(
        [[0.09, 0.18, 0.0], [0.11, 0.22, 0.0]]
    )

    adjustments = motion_compensation.recenter_grasps_on_food_binormal(
        grasps,
        food,
        [0, 1],
        np.identity(4),
    )

    assert np.allclose(adjustments, [0.1, -0.2])
    assert np.allclose(
        [x_grasp.position.x, x_grasp.position.y], [0.1, 0.0]
    )
    assert np.allclose(
        [y_grasp.position.x, y_grasp.position.y], [0.0, 0.2]
    )


def test_binormal_recentering_rejects_zero_binormal():
    grasps = GraspConfigList()
    grasps.grasps = [GraspConfig()]
    food = pointcloud_conversion.numpy_to_ros([[0.0, 0.0, 0.0]])

    with pytest.raises(
        ValueError, match='binormal must be finite and nonzero'
    ):
        motion_compensation.recenter_grasps_on_food_binormal(
            grasps,
            food,
            [0],
            np.identity(4),
        )
