"""Tests for the gripper-closing and gripper-opening behavior states."""

from types import SimpleNamespace

from robot_behavior.shared_state_data import SharedStateData
from robot_behavior.state_attach_object_to_gripper import (
    StateAttachObjectToGripper,
)
from robot_behavior.state_close_gripper import StateCloseGripper
from robot_behavior.state_open_gripper import StateOpenGripper


class FakeLogger:
    """Accept state log calls."""

    def get_child(self, _name):
        return self

    def info(self, _message):
        pass

    def debug(self, _message):
        pass

    def warning(self, _message):
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
        self.open_handlers = None
        self.attach_handlers = None
        self.clear_octomap_handlers = None
        self.unlock_base_handlers = None

    def close_gripper(self, **handlers):
        self.close_handlers = handlers

    def open_gripper(self, **handlers):
        self.open_handlers = handlers

    def attach_object_to_gripper(self, **handlers):
        self.attach_handlers = handlers

    def clear_octomap(self, **handlers):
        self.clear_octomap_handlers = handlers

    def unlock_base(self, **handlers):
        self.unlock_base_handlers = handlers


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
    assert transitions == []
    client.unlock_base_handlers['response_handler'](
        SimpleNamespace(success=True, message='')
    )
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
    assert transitions == []
    client.unlock_base_handlers['response_handler'](
        SimpleNamespace(success=True, message='')
    )
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


def test_attach_object_brings_the_arm_straight_home():
    """The arm goes home once the object is in the planning scene."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateAttachObjectToGripper(client, transitions.append)

    state.on_entry()
    client.attach_handlers['response_handler'](
        SimpleNamespace(success=True, message='')
    )

    assert transitions == ['moveToHome']


def test_attach_object_finishes_null_when_the_attachment_fails():
    """A planning scene that rejects the object ends the chain."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateAttachObjectToGripper(client, transitions.append)

    state.on_entry()
    client.attach_handlers['response_handler'](
        SimpleNamespace(success=False, message='no object')
    )

    assert transitions == [None]


def test_open_gripper_releases_the_object_and_brings_the_arm_home():
    """Opening the gripper clears the grip flag before the arm returns home."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData(object_gripped=True)
    state = StateOpenGripper(client, transitions.append, shared_data)

    state.on_entry()
    client.open_handlers['result_handler'](
        SimpleNamespace(reached_goal=True, stalled=False)
    )

    assert shared_data.object_gripped is False
    assert transitions == ['moveToHome']


def test_open_gripper_keeps_the_object_gripped_when_the_gripper_stalls():
    """A gripper that never opens still counts as holding the object."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData(object_gripped=True)
    state = StateOpenGripper(client, transitions.append, shared_data)

    state.on_entry()
    client.open_handlers['result_handler'](
        SimpleNamespace(reached_goal=False, stalled=True)
    )

    assert shared_data.object_gripped is True
    assert transitions == [None]
