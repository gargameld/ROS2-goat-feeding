"""Tests for the static grasp marker conversion."""

import math

from geometry_msgs.msg import PoseStamped
from grasp_pose_interface.msg import GraspPoseArray
from grasp_rvis.grasp_marker_node import _rotate_vector, GraspMarkerNode
import rclpy
from visualization_msgs.msg import Marker


class _CapturePublisher:
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


def test_one_pose_has_selectable_static_namespace():
    rclpy.init()
    node = GraspMarkerNode()
    try:
        capture = _CapturePublisher()
        gripper_capture = _CapturePublisher()
        node._publisher = capture
        node._gripper_publisher = gripper_capture

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 123
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        pose.pose.orientation.w = 1.0

        message = GraspPoseArray()
        message.poses = [pose]
        node._poses_callback(message)

        assert capture.message.markers[0].action == Marker.DELETEALL
        grasp_markers = capture.message.markers[1:]
        assert len(grasp_markers) == 4
        assert {marker.ns for marker in grasp_markers} == {'grasp_000'}
        assert all(marker.header.frame_id == 'map' for marker in grasp_markers)
        assert all(marker.header.stamp.sec == 0 for marker in grasp_markers)
        assert all(marker.header.stamp.nanosec == 0 for marker in grasp_markers)
        assert all(marker.lifetime.sec == 0 for marker in grasp_markers)
        assert all(marker.lifetime.nanosec == 0 for marker in grasp_markers)

        gripper_markers = gripper_capture.message.markers[1:]
        assert len(gripper_markers) == 3
        assert {marker.ns for marker in gripper_markers} == {'grasp_000'}
        assert all(marker.type == Marker.CUBE for marker in gripper_markers)
        # The real pads extend 13.9 mm beyond TCP and 23.1 mm behind it.
        # The Robotiq finger chains are separated along arm_tcp X.
        assert math.isclose(gripper_markers[0].pose.position.x, 0.955)
        assert math.isclose(gripper_markers[0].pose.position.y, 2.0)
        assert math.isclose(gripper_markers[0].pose.position.z, 2.9954)
        assert math.isclose(gripper_markers[0].scale.x, 0.012)
        assert math.isclose(gripper_markers[0].scale.y, 0.022)
        assert math.isclose(gripper_markers[0].scale.z, 0.037)
        assert math.isclose(gripper_markers[2].scale.x, 0.09)
        assert math.isclose(gripper_markers[2].scale.y, 0.022)

        # RViz discards a disabled namespace's visuals. The periodic ADD-only
        # update makes it visible again after the checkbox is re-enabled.
        capture.message = None
        gripper_capture.message = None
        node._republish()
        assert len(capture.message.markers) == 4
        assert len(gripper_capture.message.markers) == 3
        assert all(
            marker.action == Marker.ADD for marker in capture.message.markers
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rotate_vector_uses_pose_orientation():
    pose = PoseStamped()
    pose.pose.orientation.z = math.sqrt(0.5)
    pose.pose.orientation.w = math.sqrt(0.5)

    x, y, z = _rotate_vector(pose, (1.0, 0.0, 0.0))

    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 1.0, abs_tol=1e-12)
    assert math.isclose(z, 0.0, abs_tol=1e-12)
