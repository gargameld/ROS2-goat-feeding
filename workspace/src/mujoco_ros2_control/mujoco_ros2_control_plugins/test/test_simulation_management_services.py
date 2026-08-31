#!/usr/bin/env python3

"""Exercise the simulation-management obstacle services against a live graph."""

import rclpy

from mujoco_ros2_control_msgs.srv import GetRobotState
from mujoco_ros2_control_msgs.srv import SetObstacle
from mujoco_ros2_control_msgs.srv import ThrowFood


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
        initial_z_and_size = (
            initial_state.obstacle_position.z,
            initial_state.obstacle_size.x,
            initial_state.obstacle_size.y,
            initial_state.obstacle_size.z,
        )
        if initial_xy != (-2.0, -7.5):
            raise AssertionError(
                f'Expected obstacle to start in the corner, received {initial_xy}.'
            )
        request = SetObstacle.Request()
        request.position.x = 1.25
        request.position.y = -1.75
        request.position.z = 99.0
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
        expected = (1.25, -1.75, *initial_z_and_size)
        if any(abs(left - right) > 1e-9 for left, right in zip(actual, expected)):
            raise AssertionError(f'Expected {expected}, received {actual}.')
        print(f'Live obstacle state: {actual}')

        throw_client = node.create_client(
            ThrowFood, '/simulation_management/throw_food'
        )
        throw_request = ThrowFood.Request()
        throw_request.parking_index = 1
        throw_request.food_name = 'food_box'
        throw_request.x = 0.25
        throw_request.y = 0.0
        throw_request.orientation = [1.0, 0.0, 0.0, 0.0]
        throw_response = call(node, throw_client, throw_request)
        if not throw_response.success:
            raise RuntimeError(throw_response.message)
        print(f'Threw food: {throw_response.message}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
