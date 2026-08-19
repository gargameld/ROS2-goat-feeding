#!/usr/bin/env python3
"""Compare equivalent map-frame and base-link-frame arm pose goals."""

import argparse
import math
import sys
import time

from arm_interface.action import MoveArmToHomePose, MoveArmToPose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class PoseFrameComparison(Node):
    def __init__(self, args):
        super().__init__('compare_pose_frames')
        self._args = args
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._move_client = ActionClient(self, MoveArmToPose, args.action_name)
        self._home_client = ActionClient(
            self, MoveArmToHomePose, args.home_action_name)

    def send_pose(self, pose: PoseStamped, reference_frame: str) -> bool:
        goal = MoveArmToPose.Goal()
        goal.target_pose = pose.pose
        goal.reference_frame = reference_frame

        self.get_logger().info(
            f'Sending pose in {reference_frame}: '
            f'position=({pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, '
            f'{pose.pose.position.z:.4f}), '
            f'quaternion=({pose.pose.orientation.x:.6f}, {pose.pose.orientation.y:.6f}, '
            f'{pose.pose.orientation.z:.6f}, {pose.pose.orientation.w:.6f})')
        goal_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Goal in {reference_frame} was rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'Goal in {reference_frame}: success={result.success}, message={result.message}')
        return result.success

    def return_home(self) -> bool:
        goal_future = self._home_client.send_goal_async(MoveArmToHomePose.Goal())
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Home goal was rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'Home goal: success={result.success}, message={result.message}')
        return result.success

    def run(self) -> int:
        if not self._move_client.wait_for_server(timeout_sec=self._args.server_timeout):
            self.get_logger().error(f'Action server {self._args.action_name} is unavailable')
            return 1
        if self._args.home_between and not self._home_client.wait_for_server(
                timeout_sec=self._args.server_timeout):
            self.get_logger().error(
                f'Action server {self._args.home_action_name} is unavailable')
            return 1

        map_pose = PoseStamped()
        map_pose.header.frame_id = self._args.map_frame
        map_pose.pose.position.x = self._args.x
        map_pose.pose.position.y = self._args.y
        map_pose.pose.position.z = self._args.z
        map_pose.pose.orientation.x = self._args.qx
        map_pose.pose.orientation.y = self._args.qy
        map_pose.pose.orientation.z = self._args.qz
        map_pose.pose.orientation.w = self._args.qw

        transform = None
        last_error = None
        deadline = time.monotonic() + self._args.tf_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._args.default_frame, self._args.map_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.0))
                break
            except Exception as error:
                last_error = error
        if transform is None:
            self.get_logger().error(
                f'Cannot transform {self._args.map_frame} to {self._args.default_frame}: {last_error}')
            return 1

        default_pose = do_transform_pose(map_pose.pose, transform)
        default_stamped = PoseStamped()
        default_stamped.header.frame_id = self._args.default_frame
        default_stamped.pose = default_pose

        map_success = self.send_pose(map_pose, self._args.map_frame)
        if self._args.home_between and map_success:
            self.return_home()
        default_success = self.send_pose(default_stamped, self._args.default_frame)

        self.get_logger().info(
            f'Comparison complete: {self._args.map_frame}={map_success}, '
            f'{self._args.default_frame}={default_success}')
        return 0 if map_success == default_success else 2


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Send the same TCP pose in map and transformed base_link frames.')
    parser.add_argument('--x', type=float, default=0.45)
    parser.add_argument('--y', type=float, default=0.10)
    parser.add_argument('--z', type=float, default=0.60)
    parser.add_argument('--qx', type=float, default=0.0)
    parser.add_argument('--qy', type=float, default=0.0)
    parser.add_argument('--qz', type=float, default=0.0)
    parser.add_argument('--qw', type=float, default=1.0)
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--default-frame', default='base_link')
    parser.add_argument('--action-name', default='/arm/move_arm_to_pose')
    parser.add_argument('--home-action-name', default='/arm/move_arm_to_home_pose')
    parser.add_argument('--server-timeout', type=float, default=10.0)
    parser.add_argument('--tf-timeout', type=float, default=3.0)
    parser.add_argument('--home-between', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    norm = math.sqrt(args.qx ** 2 + args.qy ** 2 + args.qz ** 2 + args.qw ** 2)
    if norm == 0.0:
        parser.error('The quaternion must not be zero')
    args.qx /= norm
    args.qy /= norm
    args.qz /= norm
    args.qw /= norm
    return args


def main():
    args = parse_arguments()
    rclpy.init()
    node = PoseFrameComparison(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
