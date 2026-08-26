"""Behavior state that closes the gripper on the grasped object."""

from robot_behavior.base_state import BaseState


class StateCloseGripper(BaseState):
    """Close the gripper and record that an object is held."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Start the close-gripper action."""
        self.shared_state_data.object_gripped = False
        self.logger.info('Closing gripper on the object')
        self.behavior_client.close_gripper(
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Close-gripper goal accepted')
            return

        self.logger.error('Close-gripper goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(
            f'Gripper position: {feedback.position:.3f}; '
            f'effort: {feedback.effort:.3f}'
        )

    def _handle_result(self, result) -> None:
        if not result.reached_goal and not result.stalled:
            self.logger.error('Failed to close the gripper')
            self.request_state_transition(None)
            return

        self.shared_state_data.object_gripped = True
        self.logger.info('Gripper closed on the object')
        # The grasped object is still an obstacle in the octomap, and the
        # closed fingers now sit inside its voxels. Those voxels cannot be
        # observed as free while the gripper occludes them, so drop them
        # before the planning scene is used again.
        self.logger.info('Clearing the octomap around the gripped object')
        self.behavior_client.clear_octomap(
            response_handler=self._handle_octomap_cleared,
        )

    def _handle_octomap_cleared(self, _response) -> None:
        self.logger.info('Octomap cleared')
        self.request_state_transition('attachObjectToGripper')
