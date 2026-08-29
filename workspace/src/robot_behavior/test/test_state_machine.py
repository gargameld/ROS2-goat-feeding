"""Tests for state-machine dependency wiring."""

from robot_behavior.state_machine import StateMachine


class FakeLogger:
    """Provide child loggers to behavior states."""

    def get_child(self, _name):
        """Return this logger as the requested child."""
        return self

    def info(self, _message):
        """Accept an info log call."""


class FakeNode:
    """Provide the node interface used while constructing states."""

    def get_logger(self):
        """Return a fake logger."""
        return FakeLogger()


class FakeBehaviorClient:
    """Provide a node to behavior states."""

    def __init__(self):
        """Create the fake node."""
        self.node = FakeNode()


def test_state_machine_shares_request_listener_with_states():
    """States initialized by the machine can read the request listener."""
    request_listener = object()

    map_parameters = object()
    state_machine = StateMachine(
        FakeBehaviorClient(), request_listener, map_parameters
    )

    assert state_machine.request_listener is request_listener
    assert (
        state_machine.states['moveToHome'].request_listener
        is request_listener
    )
    assert (
        state_machine.states['waitFoodRequest'].request_listener
        is request_listener
    )
    assert state_machine.states['navigateToParking'].map_parameters is (
        map_parameters
    )
    assert state_machine.states['navigateToHole'].map_parameters is (
        map_parameters
    )
    assert state_machine.states['moveArmToHolePose'].map_parameters is (
        map_parameters
    )
    assert state_machine.states['moveToHome'].shared_state_data is (
        state_machine.shared_state_data
    )
    assert state_machine.states['findGraspPose'].shared_state_data is (
        state_machine.shared_state_data
    )
    assert state_machine.states['moveArmToPose'].shared_state_data is (
        state_machine.shared_state_data
    )
    assert state_machine.states['openGripper'].shared_state_data is (
        state_machine.shared_state_data
    )
    assert 'liftGripper' not in state_machine.states
    state_machine.change_state(None)
    assert state_machine.current_state == 'nullState'
