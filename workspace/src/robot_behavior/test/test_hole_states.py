"""Tests for the delivery states that reach the hole of a request."""

from types import SimpleNamespace

from robot_behavior.map_parameters_loader import MapPose
from robot_behavior.shared_state_data import SharedStateData
from robot_behavior.state_move_arm_to_hole_pose import StateMoveArmToHolePose
from robot_behavior.state_move_arm_to_home import StateMoveArmToHome
from robot_behavior.state_navigate_to_hole import StateNavigateToHole


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
    """Capture the requests made by the delivery states."""

    def __init__(self):
        self.node = FakeNode()
        self.navigation = None
        self.arm_pose = None
        self.home_handlers = None

    def navigate_to_pose(self, pose, **handlers):
        self.navigation = (pose, handlers)

    def move_arm_to_pose(self, pose, reference_frame, **handlers):
        self.arm_pose = (pose, reference_frame, handlers)

    def move_arm_to_home(self, **handlers):
        self.home_handlers = handlers


class FakeRequestListener:
    """Return a configurable parking number."""

    def __init__(self, parking_number=None):
        self.parking_number = parking_number

    def get_parking_number(self):
        return self.parking_number


class FakeMapParameters:
    """Return known hole targets."""

    def __init__(self):
        self.requested_hole = None
        self.requested_hole_arm = None

    def get_hole_pose(self, parking_number):
        self.requested_hole = parking_number
        return MapPose('map', -1.65, 2.1, 0.0, 0.0, 0.0, 1.0, 1.57)

    def get_hole_arm_pose(self, parking_number):
        self.requested_hole_arm = parking_number
        return MapPose('map', -2.6, 2.0, 0.42, 0.0, -0.7071068, 0.0, 0.7071068)


def test_move_arm_to_home_delivers_a_gripped_object():
    """Reaching home while holding food starts the delivery drive."""
    transitions = []
    client = FakeBehaviorClient()
    shared_data = SharedStateData(object_gripped=True)
    state = StateMoveArmToHome(
        client, transitions.append, FakeRequestListener(4), shared_data
    )

    state.on_entry()
    client.home_handlers['result_handler'](
        SimpleNamespace(success=True, message='')
    )

    assert transitions == ['navigateToHole']


def test_move_arm_to_home_waits_when_no_object_is_gripped():
    """An empty gripper still returns the robot to the request wait."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateMoveArmToHome(
        client,
        transitions.append,
        FakeRequestListener(),
        SharedStateData(),
    )

    state.on_entry()
    client.home_handlers['result_handler'](
        SimpleNamespace(success=True, message='')
    )

    assert transitions == ['waitFoodRequest']


def test_navigation_to_hole_uses_the_requested_hole_then_moves_the_arm():
    """The hole of the served parking is driven to before the arm moves."""
    transitions = []
    client = FakeBehaviorClient()
    parameters = FakeMapParameters()
    state = StateNavigateToHole(
        client,
        transitions.append,
        FakeRequestListener(4),
        parameters,
    )

    state.on_entry()

    assert parameters.requested_hole == 4
    target, handlers = client.navigation
    assert target.header.frame_id == 'map'
    assert target.pose.position.x == -1.65
    assert target.pose.position.y == 2.1
    handlers['result_handler'](object())
    assert transitions == ['moveArmToHolePose']


def test_navigation_to_hole_finishes_null_without_a_request():
    """Without a served parking there is no hole to drive to."""
    transitions = []
    client = FakeBehaviorClient()
    state = StateNavigateToHole(
        client,
        transitions.append,
        FakeRequestListener(),
        FakeMapParameters(),
    )

    state.on_entry()

    assert client.navigation is None
    assert transitions == [None]


def test_move_arm_to_hole_pose_uses_the_configured_hole_arm_target():
    """The arm is sent to the configured pose above the served hole."""
    transitions = []
    client = FakeBehaviorClient()
    parameters = FakeMapParameters()
    state = StateMoveArmToHolePose(
        client,
        transitions.append,
        FakeRequestListener(4),
        parameters,
    )

    state.on_entry()

    assert parameters.requested_hole_arm == 4
    target, reference_frame, handlers = client.arm_pose
    assert reference_frame == 'map'
    assert target.position.x == -2.6
    assert target.position.z == 0.42
    handlers['result_handler'](SimpleNamespace(success=True, message=''))
    assert transitions == ['openGripper']
