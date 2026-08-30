"""Behavior state that navigates to the requested parking area."""

from nav2_msgs.action import NavigateToPose

from robot_behavior.base_state import BaseState


class StateNavigateToParking(BaseState):
    """Navigate to the target configured for the requested parking."""

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
        """Look up the requested target and start Nav2 navigation."""
        parking_number = self.shared_state_data.parking_number
        if parking_number is None:
            self.logger.error('Cannot navigate without a food request')
            self.request_state_transition(None)
            return

        target_pose = self.map_parameters.get_parking_pose(parking_number)

        self.logger.info(f'Navigating to parking {parking_number}')
        self.behavior_client.navigate_to_pose(
            target_pose.to_pose_stamped(),
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Navigation goal accepted')
            return

        self.logger.error('Navigation goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(
            f'Distance remaining: {feedback.distance_remaining:.2f} m'
        )

    def _handle_result(self, result) -> None:
        if result.error_code != NavigateToPose.Result.NONE:
            detail = result.error_msg or f'error code {result.error_code}'
            self.logger.error(f'Parking navigation failed: {detail}')
            self.request_state_transition(None)
            return

        self.logger.info('Parking navigation finished')
        self.request_state_transition('findGraspPose')
