"""Behavior state that lifts a gripped object."""

from robot_behavior.base_state import BaseState


class StateLiftGripper(BaseState):
    """Lift the gripper using the arm subsystem's lift action."""

    def on_entry(self) -> None:
        """Start the lift-gripper action."""
        self.logger.info('Lifting the gripped object')
        self.behavior_client.lift_gripper(
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Lift-gripper goal accepted')
            return

        self.logger.error('Lift-gripper goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Lift-gripper action state: {feedback.state}')

    def _handle_result(self, result) -> None:
        if result.success:
            self.logger.info('Gripped object lifted')
        else:
            self.logger.error(f'Failed to lift the gripper: {result.message}')

        self.request_state_transition(None)
