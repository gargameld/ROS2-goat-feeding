"""Behavior state that opens the gripper to release the carried object."""

from robot_behavior.base_state import BaseState


class StateOpenGripper(BaseState):
    """Open the gripper and detach its payload from the planning scene."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Start the open-gripper action."""
        self.logger.info('Opening gripper to release the object')
        self.behavior_client.open_gripper(
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Open-gripper goal accepted')
            return

        self.logger.error('Open-gripper goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(
            f'Gripper position: {feedback.position:.3f}; '
            f'effort: {feedback.effort:.3f}'
        )

    def _handle_result(self, result) -> None:
        if not result.reached_goal:
            self.logger.error('Failed to open the gripper')
            self.request_state_transition(None)
            return

        self.shared_state_data.object_gripped = False
        self.logger.info(
            'Gripper opened; detaching the payload from the planning scene'
        )
        self.behavior_client.detach_object_from_gripper(
            response_handler=self._handle_detach_response,
        )

    def _handle_detach_response(self, response) -> None:
        if not response.success:
            self.logger.error(
                'Failed to detach payload from the planning scene: '
                f'{response.message}'
            )
            self.request_state_transition(None)
            return

        self.shared_state_data.parking_number = None
        self.logger.info('Payload detached and the object released')
        self.request_state_transition('moveToHome')
