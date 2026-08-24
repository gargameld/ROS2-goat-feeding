"""Behavior state that waits for a parking-specific food request."""

from robot_behavior.base_state import BaseState


class StateWaitFoodRequest(BaseState):
    """Wait until the request listener contains a parking number."""

    def on_entry(self) -> None:
        """Report that the behavior is ready for a request."""
        self.logger.info('Waiting for a food request')

    def tick(self) -> None:
        """Start parking navigation once a request is available."""
        if self.request_listener.get_parking_number() is not None:
            self.request_state_transition('navigateToParking')
