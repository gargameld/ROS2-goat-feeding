"""Tests for the gripper-closing and object-lifting behavior states."""

from types import SimpleNamespace

from robot_behavior.shared_state_data import SharedStateData
from robot_behavior.state_close_gripper import StateCloseGripper
from robot_behavior.state_lift_gripper import StateLiftGripper


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
    """Capture the gripper action and service calls made by the states."""

    def __init__(self):
        self.node = FakeNode()
        self.close_handlers = None
        self.lift_handlers = None
        self.clear_octomap_handlers = None

    def close_gripper(self, **handlers):
        self.close_handlers = handlers

    def lift_gripper(self, **handlers):
        self.lift_handlers = handlers

    def clear_octomap(self, **handlers):
        self.clear_octomap_handlers = handlers


def test_close_gripper_marks_the_object_gripped_and_clears_the_octomap():
    """A gripper that reaches its goal clears the stale food voxels."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData()
    state = StateCloseGripper(client, transitions.append, shared_data)

    state.on_entry()
    client.close_handlers['result_handler'](
        SimpleNamespace(reached_goal=True, stalled=False)
    )

    assert shared_data.object_gripped is True
    assert transitions == []
    client.clear_octomap_handlers['response_handler'](SimpleNamespace())
    assert transitions == ['attachObjectToGripper']


def test_close_gripper_accepts_a_stall_as_a_successful_grip():
    """Stalling on an object counts as holding it."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData()
    state = StateCloseGripper(client, transitions.append, shared_data)

    state.on_entry()
    client.close_handlers['result_handler'](
        SimpleNamespace(reached_goal=False, stalled=True)
    )

    assert shared_data.object_gripped is True
    client.clear_octomap_handlers['response_handler'](SimpleNamespace())
    assert transitions == ['attachObjectToGripper']


def test_close_gripper_finishes_null_when_the_grip_fails():
    """A gripper that neither closes nor stalls ends the chain."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData()
    state = StateCloseGripper(client, transitions.append, shared_data)

    state.on_entry()
    client.close_handlers['result_handler'](
        SimpleNamespace(reached_goal=False, stalled=False)
    )

    assert shared_data.object_gripped is False
    assert transitions == [None]


def test_lift_gripper_finishes_null_after_a_successful_lift():
    """Lifting is currently the last step of the grasp chain."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateLiftGripper(client, transitions.append)

    state.on_entry()
    client.lift_handlers['result_handler'](
        SimpleNamespace(success=True, message='')
    )

    assert transitions == [None]


def test_lift_gripper_finishes_null_when_the_lift_fails():
    """A failed lift also ends the chain."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateLiftGripper(client, transitions.append)

    state.on_entry()
    client.lift_handlers['result_handler'](
        SimpleNamespace(success=False, message='no plan')
    )

    assert transitions == [None]
