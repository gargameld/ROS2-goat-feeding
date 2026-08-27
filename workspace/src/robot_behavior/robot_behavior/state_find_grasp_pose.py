"""Behavior state that requests a reachable food grasp pose."""

import time

from robot_behavior.base_state import BaseState


class StateFindGraspPose(BaseState):
    """Find a grasp pose and save it for the arm-motion state."""

    def __init__(
        self,
        behavior_client,
        request_state_transition,
        shared_state_data,
    ):
        super().__init__(behavior_client, request_state_transition)
        self.shared_state_data = shared_state_data

    def on_entry(self) -> None:
        """Pin the base, then clear any previous grasp and request a new one."""
        time.sleep(40)
        self.shared_state_data.grasp_pose = None
        self.shared_state_data.grasp_reference_frame = ''
        # The grasp pose is captured in the camera frame and handed to the arm
        # relative to the base. Any chassis drift between the capture and the
        # grasp invalidates it, so the base stays pinned from here until the
        # gripper has closed on the food.
        self.logger.info('Locking the robot base before looking for a grasp')
        self.behavior_client.lock_base(
            response_handler=self._handle_base_locked,
        )

    def _handle_base_locked(self, response) -> None:
        if not response.success:
            self.logger.error(f'Failed to lock the robot base: {response.message}')
            self.request_state_transition(None)
            return

        self.logger.info('Finding a reachable grasp pose')
        self.behavior_client.provide_grasp_pose(
            goal_response_handler=self._handle_goal_response,
            feedback_handler=self._handle_feedback,
            result_handler=self._handle_result,
        )

    def _handle_goal_response(self, goal_handle) -> None:
        if goal_handle.accepted:
            self.logger.info('Grasp-pose goal accepted')
            return

        self.logger.error('Grasp-pose goal rejected')
        self.request_state_transition(None)

    def _handle_feedback(self, feedback) -> None:
        self.logger.debug(f'Grasp-pose action stage: {feedback.stage}')

    def _handle_result(self, result) -> None:
        if not result.food_found:
            self.logger.error('No reachable food grasp pose was found')
            self.request_state_transition(None)
            return

        self.shared_state_data.grasp_pose = result.grasp_pose
        self.shared_state_data.grasp_reference_frame = result.reference_frame
        self.logger.info('A reachable grasp pose was found')
        self.request_state_transition('moveArmToPose')
