"""Unit tests for the GUI-facing MuJoCo ROS client."""

from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from simulation_interface_gui.ros.mujoco_client import MujocoClient
from simulation_interface_gui.ros.mujoco_client import ServiceUnavailableError


class FakePublisher:
    """Record published messages."""

    def __init__(self):
        """Create an empty message list."""
        self.messages = []

    def publish(self, message):
        """Record one message."""
        self.messages.append(message)


class FakeServiceClient:
    """Return a configurable response without ROS middleware."""

    def __init__(self):
        """Create an available fake service."""
        self.available = True
        self.response = SimpleNamespace(qpos=[1, 2.5, -3])

    def service_is_ready(self):
        """Return the configured availability."""
        return self.available

    def call_async(self, _request):
        """Return an already-completed response future."""
        future = Future()
        future.set_result(self.response)
        return future


class FakeNode:
    """Provide the node operations used by ``MujocoClient``."""

    def __init__(self):
        """Create fake ROS entities."""
        self.publisher = FakePublisher()
        self.client = FakeServiceClient()

    def create_publisher(self, *_args):
        """Return the fake publisher."""
        return self.publisher

    def create_client(self, *_args):
        """Return the fake service client."""
        return self.client

    def destroy_publisher(self, _publisher):
        """Accept publisher destruction."""

    def destroy_client(self, _client):
        """Accept client destruction."""


class ImmediateRuntime:
    """Execute submitted callbacks immediately for deterministic tests."""

    def __init__(self):
        """Create the fake node."""
        self.node = FakeNode()

    def submit(self, callback):
        """Execute a callback and represent its outcome as a future."""
        future = Future()
        try:
            future.set_result(callback())
        except BaseException as error:
            future.set_exception(error)
        return future


@pytest.fixture
def client_and_runtime():
    """Create a client attached to the immediate runtime."""
    runtime = ImmediateRuntime()
    return MujocoClient(runtime), runtime


def test_change_cmd_vel_maps_all_twist_fields(client_and_runtime):
    """The six GUI velocity fields are mapped to one Twist message."""
    client, runtime = client_and_runtime

    client.change_cmd_vel(1, 2, 3, 4, 5, 6).result()

    message = runtime.node.publisher.messages[-1]
    assert (message.linear.x, message.linear.y, message.linear.z) == (1, 2, 3)
    assert (message.angular.x, message.angular.y, message.angular.z) == (4, 5, 6)


def test_change_cmd_vel_rejects_non_finite_values(client_and_runtime):
    """Invalid textbox values are rejected before publishing."""
    client, _runtime = client_and_runtime

    with pytest.raises(ValueError):
        client.change_cmd_vel(linear_x=float('nan'))


def test_get_robot_state_returns_float_list(client_and_runtime):
    """The service qpos sequence is exposed as a plain float list."""
    client, _runtime = client_and_runtime

    assert client.get_robot_state().result() == [1.0, 2.5, -3.0]


def test_get_robot_state_reports_unavailable_service(client_and_runtime):
    """An unavailable state service completes with a useful exception."""
    client, runtime = client_and_runtime
    runtime.node.client.available = False

    with pytest.raises(ServiceUnavailableError):
        client.get_robot_state().result()
