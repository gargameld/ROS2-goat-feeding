"""Behavior state that waits for a parking-specific food request."""

from robot_behavior.base_state import BaseState


class StateWaitFoodRequest(BaseState):
    """Wait until shared state contains a parking number."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Report that the behavior is ready for a request."""
        self.logger.info('Waiting for a food request')

    def tick(self) -> None:
        """Start parking navigation once a request is available."""
        if self.shared_state_data.parking_number is not None:
            self.request_state_transition('navigateToParking')
