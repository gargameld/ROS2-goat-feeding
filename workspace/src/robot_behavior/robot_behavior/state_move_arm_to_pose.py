"""Behavior state that moves the arm to the shared grasp pose."""

from robot_behavior.base_state import BaseState


class StateMoveArmToPose(BaseState):
    """Move the arm to the grasp pose produced by the preceding state."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Start moving the arm to the stored grasp pose."""
        target_pose = self.shared_state_data.grasp_pose
        if target_pose is None:
            self.logger.error('Cannot move the arm without a grasp pose')
            self.request_state_transition(None)
            return

        self.logger.info('Moving arm to the grasp pose')
        self.behavior_client.move_arm_to_pose(
            target_pose,
            self.shared_state_data.grasp_reference_frame,
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Move-to-pose goal accepted')
            return

        self.logger.error('Move-to-pose goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Move-to-pose action state: {feedback.state}')

    def _handle_result(self, result) -> None:
        if result.success:
            self.logger.info('Arm moved to the grasp pose')
        else:
            self.logger.error(
                f'Failed to move arm to the grasp pose: {result.message}'
            )

        self.request_state_transition(None)
