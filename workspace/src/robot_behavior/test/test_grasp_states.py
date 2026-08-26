"""Tests for grasp-pose lookup and arm-motion behavior states."""

from types import SimpleNamespace

from geometry_msgs.msg import Pose

import robot_behavior.state_find_grasp_pose as find_grasp_pose_module
from robot_behavior.shared_state_data import SharedStateData
from robot_behavior.state_find_grasp_pose import StateFindGraspPose
from robot_behavior.state_move_arm_to_pose import StateMoveArmToPose


class FakeLogger:
    """Accept state log calls."""

    def get_child(self, _name):
        return self

    def info(self, _message):
        pass

    def debug(self, _message):
        pass

    def error(self, _message):
        pass


class FakeNode:
    """Provide logging to states."""

    def get_logger(self):
        return FakeLogger()


class FakeBehaviorClient:
    """Capture action calls made by the states."""

    def __init__(self):
        self.node = FakeNode()
        self.grasp_handlers = None
        self.move_request = None

    def provide_grasp_pose(self, **handlers):
        self.grasp_handlers = handlers

    def move_arm_to_pose(self, pose, reference_frame, **handlers):
        self.move_request = (pose, reference_frame, handlers)


def test_find_grasp_pose_stores_result_and_transitions_to_arm_motion(
    monkeypatch,
):
    """The grasp result is passed to the next state through shared data."""
    sleep_calls = []
    monkeypatch.setattr(find_grasp_pose_module.time, 'sleep', sleep_calls.append)
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData()
    state = StateFindGraspPose(client, transitions.append, shared_data)

    state.on_entry()
    assert sleep_calls == [20]
    pose = Pose()
    pose.position.x = 1.2
    client.grasp_handlers['result_handler'](
        SimpleNamespace(
            food_found=True,
            grasp_pose=pose,
            reference_frame='base_link',
        )
    )

    assert shared_data.grasp_pose is pose
    assert shared_data.grasp_reference_frame == 'base_link'
    assert transitions == ['moveArmToPose']


def test_find_grasp_pose_finishes_null_when_food_is_not_found(monkeypatch):
    """An unsuccessful grasp lookup does not start arm motion."""
    monkeypatch.setattr(find_grasp_pose_module.time, 'sleep', lambda _seconds: None)
    transitions = []
    client = FakeBehaviorClient()
    state = StateFindGraspPose(
        client,
        transitions.append,
        SharedStateData(),
    )

    state.on_entry()
    client.grasp_handlers['result_handler'](
        SimpleNamespace(food_found=False)
    )

    assert transitions == [None]


def test_move_arm_uses_shared_grasp_and_closes_the_gripper():
    """Arm motion consumes the stored pose and hands over to the gripper."""
    transitions = []
    client = FakeBehaviorClient()
    pose = Pose()
    shared_data = SharedStateData(pose, 'base_link')
    state = StateMoveArmToPose(client, transitions.append, shared_data)

    state.on_entry()

    requested_pose, reference_frame, handlers = client.move_request
    assert requested_pose is pose
    assert reference_frame == 'base_link'
    handlers['result_handler'](SimpleNamespace(success=True, message=''))
    assert transitions == ['closeGripper']
