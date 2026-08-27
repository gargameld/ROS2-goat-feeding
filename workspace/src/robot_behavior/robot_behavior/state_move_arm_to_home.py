"""Behavior state that moves the arm to its home pose."""

from robot_behavior.base_state import BaseState


class StateMoveArmToHome(BaseState):
    """Move the arm home, then either deliver the object or wait."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        request_listener,
        shared_state_data,
    ):
        super().__init__(
            behavior_client,
            request_state_transition,
            request_listener,
        )
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Start the move-to-home action."""
        self.logger.info('Moving arm to home pose')
        self.behavior_client.move_arm_to_home(
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Move-to-home goal accepted')
            return

        self.logger.error('Move-to-home goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Move-to-home action state: {feedback.state}')

    def _handle_result(self, result) -> None:
        if result.success:
            self.logger.info('Arm moved to home pose')
            self.request_state_transition(self._next_state())
            return

        self.logger.error(
            f'Failed to move arm to home pose: {result.message}'
        )
        self.request_state_transition(None)

    def _next_state(self) -> str:
        """Deliver a gripped object; otherwise wait for the next request."""
        if self.shared_state_data.object_gripped:
            return 'navigateToHole'

        return 'waitFoodRequest'
