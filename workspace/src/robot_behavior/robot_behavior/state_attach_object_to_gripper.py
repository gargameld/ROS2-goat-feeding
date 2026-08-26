"""Behavior state that represents the held object in the planning scene."""

from robot_behavior.base_state import BaseState


class StateAttachObjectToGripper(BaseState):
    """Attach the configured payload box to the gripper in MoveIt."""

    def on_entry(self) -> None:
        """Call the existing planning-scene attachment service."""
        self.logger.info('Adding the gripped object to the planning scene')
        self.behavior_client.attach_object_to_gripper(
            response_handler=self._handle_response,
        )

    def _handle_response(self, response) -> None:
        if response.success:
            self.logger.info('Object attached to the gripper in the planning scene')
            self.request_state_transition('liftGripper')
            return

        self.logger.error(
            f'Failed to attach object in the planning scene: {response.message}'
        )
        self.request_state_transition(None)
