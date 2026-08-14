#!/usr/bin/env python3
# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Exercise the simulation-management obstacle services against a live graph."""

import rclpy

from mujoco_ros2_control_msgs.srv import GetRobotState
from mujoco_ros2_control_msgs.srv import SetObstacle


def call(node, client, request):
    """Wait for and invoke one ROS service."""
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'Service {client.srv_name} is unavailable.')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    if not future.done():
        raise TimeoutError(f'Service {client.srv_name} timed out.')
    return future.result()


def main():
    """Update the box and verify the state service reports the compiled result."""
    rclpy.init()
    node = rclpy.create_node('test_simulation_management_services')
    try:
        set_client = node.create_client(
            SetObstacle, '/simulation_management/set_obstacle'
        )
        state_client = node.create_client(
            GetRobotState, '/simulation_management/get_robot_state'
        )
        initial_state = call(node, state_client, GetRobotState.Request())
        initial_xy = (
            initial_state.obstacle_position.x,
            initial_state.obstacle_position.y,
        )
        if initial_xy != (-2.0, -7.5):
            raise AssertionError(
                f'Expected obstacle to start in the corner, received {initial_xy}.'
            )
        request = SetObstacle.Request()
        request.position.x = 1.25
        request.position.y = -1.75
        request.position.z = 99.0
        request.size.x = 0.6
        request.size.y = 1.4
        request.size.z = 1.2
        response = call(node, set_client, request)
        if not response.success:
            raise RuntimeError(response.message)

        state = call(node, state_client, GetRobotState.Request())
        actual = (
            state.obstacle_position.x,
            state.obstacle_position.y,
            state.obstacle_position.z,
            state.obstacle_size.x,
            state.obstacle_size.y,
            state.obstacle_size.z,
        )
        expected = (1.25, -1.75, 0.6, 0.6, 1.4, 1.2)
        if any(abs(left - right) > 1e-9 for left, right in zip(actual, expected)):
            raise AssertionError(f'Expected {expected}, received {actual}.')
        print(f'Live obstacle state: {actual}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
