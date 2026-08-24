"""Executable entry point for the robot behavior node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from robot_behavior.behavior_client import BehaviorClient
from robot_behavior.map_parameters_loader import MapParametersLoader
from robot_behavior.request_listener import RequestListener
from robot_behavior.state_machine import StateMachine


DEFAULT_TICK_PERIOD_SECONDS = 0.1


class RobotBehaviorNode(Node):
    """ROS node that periodically advances the behavior state machine."""

    def __init__(self):
        """Create the behavior node and its tick-period parameter."""
        super().__init__('behavior_node')
        self.declare_parameter(
            'tick_period_seconds', DEFAULT_TICK_PERIOD_SECONDS
        )


def main(args=None):
    """Initialize and run the robot behavior state machine."""
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    behavior_client = BehaviorClient(node)
    request_listener = RequestListener(node)
    configuration_file = (
        Path(get_package_share_directory('robot_behavior'))
        / 'config'
        / 'map_parameters.yaml'
    )
    map_parameters = MapParametersLoader(configuration_file)
    state_machine = StateMachine(
        behavior_client,
        request_listener,
        map_parameters,
    )
    state_machine.change_state('moveToHome')
    tick_period = (
        node.get_parameter('tick_period_seconds')
        .get_parameter_value()
        .double_value
    )
    timer = node.create_timer(tick_period, state_machine.tick)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
