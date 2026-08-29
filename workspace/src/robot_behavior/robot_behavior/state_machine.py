"""State-machine lifecycle management for the robot behavior node."""

from robot_behavior.behavior_client import BehaviorClient
from robot_behavior.map_parameters_loader import MapParametersLoader
from robot_behavior.request_listener import RequestListener
from robot_behavior.shared_state_data import SharedStateData
from robot_behavior.state_attach_object_to_gripper import (
    StateAttachObjectToGripper,
)
from robot_behavior.state_close_gripper import StateCloseGripper
from robot_behavior.state_find_grasp_pose import StateFindGraspPose
from robot_behavior.state_move_arm_to_hole_pose import StateMoveArmToHolePose
from robot_behavior.state_move_arm_to_home import StateMoveArmToHome
from robot_behavior.state_move_arm_to_pose import StateMoveArmToPose
from robot_behavior.state_navigate_to_hole import StateNavigateToHole
from robot_behavior.state_navigate_to_parking import StateNavigateToParking
from robot_behavior.state_open_gripper import StateOpenGripper
from robot_behavior.state_wait_food_request import StateWaitFoodRequest


class StateMachine:
    """Own the current behavior state and drive its lifecycle."""

    def __init__(
        self,
        behavior_client: BehaviorClient,
        request_listener: RequestListener,
        map_parameters: MapParametersLoader,
    ):
        """Create states and the data object shared between relevant states."""
        self.behavior_client = behavior_client
        self.request_listener = request_listener
        self.map_parameters = map_parameters
        self.shared_state_data = SharedStateData()
        self.states = {
            'nullState': None,
            'moveToHome': StateMoveArmToHome(
                behavior_client,
                self.change_state,
                request_listener,
                self.shared_state_data,
            ),
            'waitFoodRequest': StateWaitFoodRequest(
                behavior_client,
                self.change_state,
                request_listener,
            ),
            'navigateToParking': StateNavigateToParking(
                behavior_client,
                self.change_state,
                request_listener,
                map_parameters,
            ),
            'findGraspPose': StateFindGraspPose(
                behavior_client,
                self.change_state,
                self.shared_state_data,
            ),
            'moveArmToPose': StateMoveArmToPose(
                behavior_client,
                self.change_state,
                self.shared_state_data,
            ),
            'closeGripper': StateCloseGripper(
                behavior_client,
                self.change_state,
                self.shared_state_data,
            ),
            'attachObjectToGripper': StateAttachObjectToGripper(
                behavior_client,
                self.change_state,
            ),
            'navigateToHole': StateNavigateToHole(
                behavior_client,
                self.change_state,
                request_listener,
                map_parameters,
            ),
            'moveArmToHolePose': StateMoveArmToHolePose(
                behavior_client,
                self.change_state,
                request_listener,
                map_parameters,
            ),
            'openGripper': StateOpenGripper(
                behavior_client,
                self.change_state,
                self.shared_state_data,
            ),
        }
        self.current_state = 'nullState'

    def change_state(self, next_state) -> None:
        """Leave the current state and enter ``next_state``."""
        if self.states[self.current_state] is not None:
            self.states[self.current_state].on_exit()

        self.current_state = 'nullState' if next_state is None else next_state

        if self.states[self.current_state] is not None:
            self.states[self.current_state].on_entry()

    def tick(self) -> None:
        """Tick the current state, if one has been selected."""
        if self.states[self.current_state] is not None:
            self.states[self.current_state].tick()
