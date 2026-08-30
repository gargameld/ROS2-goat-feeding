"""Tests for request waiting and parking navigation states."""

from types import SimpleNamespace

from robot_behavior.map_parameters_loader import MapPose
from robot_behavior.state_navigate_to_parking import StateNavigateToParking
from robot_behavior.state_wait_food_request import StateWaitFoodRequest


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
    """Capture navigation requests."""

    def __init__(self):
        self.node = FakeNode()
        self.navigation = None

    def navigate_to_pose(self, pose, **handlers):
        self.navigation = (pose, handlers)


class FakeRequestListener:
    """Return a configurable parking number."""

    def __init__(self, parking_number=None):
        self.parking_number = parking_number

    def get_parking_number(self):
        return self.parking_number


class FakeMapParameters:
    """Return a known navigation target."""

    def __init__(self):
        self.requested_parking = None

    def get_parking_pose(self, parking_number):
        self.requested_parking = parking_number
        return MapPose('map', 1.95, -3.0, 0.0, 0.0, 0.0, -0.7, 0.7)


def test_wait_state_transitions_only_after_request_arrives():
    """The wait state's tick polls the shared request listener."""
    transitions = []
    listener = FakeRequestListener()
    state = StateWaitFoodRequest(
        FakeBehaviorClient(), transitions.append, listener
    )

    state.tick()
    assert transitions == []

    listener.parking_number = 3
    state.tick()
    assert transitions == ['navigateToParking']


def test_navigation_state_uses_requested_pose_then_finds_grasp_pose():
    """Navigation sends the configured target then starts grasp detection."""
    transitions = []
    client = FakeBehaviorClient()
    parameters = FakeMapParameters()
    state = StateNavigateToParking(
        client,
        transitions.append,
        FakeRequestListener(3),
        parameters,
    )

    state.on_entry()

    assert parameters.requested_parking == 3
    target, handlers = client.navigation
    assert target.header.frame_id == 'map'
    assert target.pose.position.x == 1.95
    assert target.pose.position.y == -3.0
    handlers['result_handler'](SimpleNamespace(error_code=0, error_msg=''))
    assert transitions == ['findGraspPose']


def test_navigation_failure_enters_null_state():
    """A failed Nav2 action must not continue to grasp detection."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateNavigateToParking(
        client,
        transitions.append,
        FakeRequestListener(3),
        FakeMapParameters(),
    )

    state.on_entry()
    _, handlers = client.navigation
    handlers['result_handler'](
        SimpleNamespace(error_code=106, error_msg='No valid control')
    )

    assert transitions == [None]
