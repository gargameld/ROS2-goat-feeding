"""Behavior state that navigates to the hole serving the requested parking."""

from robot_behavior.base_state import BaseState


class StateNavigateToHole(BaseState):
    """Drive the base to the hole configured for the requested parking."""

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
        """Look up the hole of the request and start Nav2 navigation."""
        parking_number = self.shared_state_data.parking_number
        if parking_number is None:
            self.logger.error('Cannot navigate without a food request')
            self.request_state_transition(None)
            return

        target_pose = self.map_parameters.get_hole_pose(parking_number)

        self.logger.info(f'Navigating to the hole of parking {parking_number}')
        self.behavior_client.navigate_to_pose(
            target_pose.to_pose_stamped(),
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Hole navigation goal accepted')
            return

        self.logger.error('Hole navigation goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(
            f'Distance remaining: {feedback.distance_remaining:.2f} m'
        )

    def _handle_result(self, _result) -> None:
        self.logger.info('Hole navigation finished')
        self.request_state_transition('moveArmToHolePose')
