"""Tests for converting GPD hand frames into the robot TCP convention."""

import math

from geometry_msgs.msg import Vector3
from gpd_ros2_msgs.msg import GraspConfig, GraspConfigList

from grasp_pose_provider.grasp_config_conversion import grasp_configs_to_poses
import numpy as np


def test_moves_from_finger_base_to_tcp_and_maps_approach_to_positive_z():
    grasp = GraspConfig()
    grasp.position.x = 1.0
    grasp.position.y = 2.0
    grasp.position.z = 3.0
    # This GPD frame maps to an identity TCP frame:
    # [binormal, axis, approach] = [X, Y, Z].
    grasp.approach = Vector3(x=0.0, y=0.0, z=1.0)
    grasp.binormal = Vector3(x=1.0, y=0.0, z=0.0)
    grasp.axis = Vector3(x=0.0, y=1.0, z=0.0)

    grasp_list = GraspConfigList()
    grasp_list.header.frame_id = 'left_camera_frame'
    grasp_list.grasps = [grasp]

    pose = grasp_configs_to_poses(
        grasp_list, tcp_from_finger_base=0.0231
    )[0]

    assert pose.header.frame_id == 'left_camera_frame'
    assert math.isclose(pose.pose.position.x, 1.0)
    assert math.isclose(pose.pose.position.y, 2.0)
    assert math.isclose(pose.pose.position.z, 3.0231)
    assert math.isclose(pose.pose.orientation.x, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.y, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.z, 0.0, abs_tol=1e-12)
    assert math.isclose(pose.pose.orientation.w, 1.0, abs_tol=1e-12)


def test_rejects_negative_tcp_offset():
    grasp_list = GraspConfigList()

    try:
        grasp_configs_to_poses(grasp_list, tcp_from_finger_base=-0.001)
    except ValueError as error:
        assert str(error) == 'tcp_from_finger_base must be non-negative.'
    else:
        raise AssertionError('negative tcp_from_finger_base should fail')


def test_anchors_pose_in_target_frame_with_capture_time_transform():
    grasp = GraspConfig()
    grasp.position.x = 1.0
    grasp.approach.z = 1.0
    grasp.binormal.x = 1.0
    grasp.axis.y = 1.0
    grasp_list = GraspConfigList()
    grasp_list.header.frame_id = 'left_camera_frame'
    grasp_list.header.stamp.sec = 42
    grasp_list.grasps = [grasp]

    # Rotate camera +X to base +Y, then translate by (10, 20, 30).
    target_from_camera = np.array(
        [
            [0.0, -1.0, 0.0, 10.0],
            [1.0, 0.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    pose = grasp_configs_to_poses(
        grasp_list,
        tcp_from_finger_base=0.0,
        target_from_grasp_frame=target_from_camera,
        target_frame='base_link',
    )[0]

    assert pose.header.frame_id == 'base_link'
    assert pose.header.stamp.sec == 42
    assert math.isclose(pose.pose.position.x, 10.0)
    assert math.isclose(pose.pose.position.y, 21.0)
    assert math.isclose(pose.pose.position.z, 30.0)
    assert math.isclose(pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(pose.pose.orientation.w, math.sqrt(0.5))
