"""Tests for converting GPD hand frames into the robot TCP convention."""

import math

from geometry_msgs.msg import Vector3
from gpd_ros2_msgs.msg import GraspConfig, GraspConfigList

from grasp_pose_provider.grasp_config_conversion import grasp_configs_to_poses


def test_moves_from_finger_base_to_tcp_and_maps_approach_to_positive_z():
    grasp = GraspConfig()
    grasp.position.x = 1.0
    grasp.position.y = 2.0
    grasp.position.z = 3.0
    # This GPD frame maps to an identity TCP frame:
    # [-axis, binormal, approach] = [X, Y, Z].
    grasp.approach = Vector3(x=0.0, y=0.0, z=1.0)
    grasp.binormal = Vector3(x=0.0, y=1.0, z=0.0)
    grasp.axis = Vector3(x=-1.0, y=0.0, z=0.0)

    grasp_list = GraspConfigList()
    grasp_list.header.frame_id = 'left_camera_frame'
    grasp_list.grasps = [grasp]

    pose = grasp_configs_to_poses(grasp_list, hand_depth=0.037)[0]

    assert pose.header.frame_id == 'left_camera_frame'
    assert math.isclose(pose.pose.position.x, 1.0)
    assert math.isclose(pose.pose.position.y, 2.0)
    assert math.isclose(pose.pose.position.z, 3.037)
    assert math.isclose(pose.pose.orientation.x, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.y, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.z, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.w, 1.0, abs_tol=1e-12)


def test_rejects_negative_hand_depth():
    grasp_list = GraspConfigList()

    try:
        grasp_configs_to_poses(grasp_list, hand_depth=-0.001)
    except ValueError as error:
        assert str(error) == 'hand_depth must be non-negative.'
    else:
        raise AssertionError('negative hand_depth should fail')
