"""Behavior state that moves the arm to its home pose."""

from robot_behavior.base_state import BaseState


class StateMoveArmToHome(BaseState):
    """Move the arm home, then wait for a food request."""

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
        self.request_state_transition('waitFoodRequest')

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Move-to-home action state: {feedback.state}')

    def _handle_result(self, result) -> None:
        if result.success:
            self.logger.info('Arm moved to home pose')
        else:
            self.logger.error(
                f'Failed to move arm to home pose: {result.message}'
            )

        self.request_state_transition('waitFoodRequest')
