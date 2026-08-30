"""Base class for robot behavior states."""

from typing import Callable, Optional

from robot_behavior.behavior_client import BehaviorClient


StateTransitionRequest = Callable[[Optional['BaseState']], None]


class BaseState:
    """Provide the shared context and lifecycle hooks for behavior states."""

    def __init__(
        self,
        behavior_client: BehaviorClient,
        request_state_transition: StateTransitionRequest,
    ):
        """Store the clients shared by behavior states."""
        self.behavior_client = behavior_client
        self.logger = behavior_client.node.get_logger().get_child(
            type(self).__name__
        )
        self.request_state_transition = request_state_transition

    def on_entry(self) -> None:
        """Run when this state becomes current."""

    def tick(self) -> None:
        """Advance this state by one state-machine tick."""

    def on_exit(self) -> None:
        """Run immediately before this state stops being current."""
