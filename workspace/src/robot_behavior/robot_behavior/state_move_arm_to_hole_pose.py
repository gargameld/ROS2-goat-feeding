"""Behavior state that moves the arm above the hole of the request."""

from robot_behavior.base_state import BaseState


class StateMoveArmToHolePose(BaseState):
    """Move the arm to the pose configured above the requested hole."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
        map_parameters,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data
        self.map_parameters = map_parameters

    def on_entry(self) -> None:
        """Start moving the arm to the configured hole pose."""
        parking_number = self.shared_state_data.parking_number
        if parking_number is None:
            self.logger.error('Cannot move the arm without a food request')
            self.request_state_transition(None)
            return

        target_pose = self.map_parameters.get_hole_arm_pose(parking_number)

        self.logger.info(
            f'Moving arm to the hole pose of parking {parking_number}'
        )
        self.behavior_client.move_arm_to_pose(
            target_pose.to_pose(),
            target_pose.frame_id,
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Move-to-hole-pose goal accepted')
            return

        self.logger.error('Move-to-hole-pose goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Move-to-hole-pose action state: {feedback.state}')

    def _handle_result(self, result) -> None:
        if result.success:
            self.logger.info('Arm moved to the hole pose')
            self.request_state_transition('openGripper')
            return

        self.logger.error(
            f'Failed to move arm to the hole pose: {result.message}'
        )
        self.request_state_transition(None)
