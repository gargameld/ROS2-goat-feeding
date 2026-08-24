"""Behavior state that navigates to the requested parking area."""

from geometry_msgs.msg import PoseStamped

from robot_behavior.base_state import BaseState


class StateNavigateToParking(BaseState):
    """Navigate to the target configured for the requested parking."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        request_listener,
        map_parameters,
    ):
        super().__init__(
            behavior_client,
            request_state_transition,
            request_listener,
        )
        self.map_parameters = map_parameters

    def on_entry(self) -> None:
        """Look up the requested target and start Nav2 navigation."""
        parking_number = self.request_listener.get_parking_number()
        if parking_number is None:
            self.logger.error('Cannot navigate without a food request')
            self.request_state_transition(None)
            return

        configured_pose = self.map_parameters.get_parking_pose(parking_number)
        target_pose = PoseStamped()
        target_pose.header.frame_id = configured_pose.frame_id
        target_pose.pose.position.x = configured_pose.x
        target_pose.pose.position.y = configured_pose.y
        target_pose.pose.position.z = configured_pose.z
        target_pose.pose.orientation.x = configured_pose.qx
        target_pose.pose.orientation.y = configured_pose.qy
        target_pose.pose.orientation.z = configured_pose.qz
        target_pose.pose.orientation.w = configured_pose.qw

        self.logger.info(f'Navigating to parking {parking_number}')
        self.behavior_client.navigate_to_pose(
            target_pose,
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

    def _handle_result(self, _result) -> None:
        self.logger.info('Parking navigation finished')
        self.request_state_transition('findGraspPose')
