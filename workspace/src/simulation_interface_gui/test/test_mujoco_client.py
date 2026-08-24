"""Unit tests for the GUI-facing MuJoCo ROS client."""

from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import ThrowFoodCommand
from simulation_interface_gui.ros.mujoco_client import MujocoClient
from simulation_interface_gui.ros.mujoco_client import ServiceUnavailableError


class FakeServiceClient:
    """Return a configurable response without ROS middleware."""

    def __init__(self):
        """Create an available fake service."""
        self.available = True
        self.response = SimpleNamespace(
            qpos=[1, 2.5, -3],
            obstacle_position=SimpleNamespace(x=4, y=5, z=0.5),
            obstacle_size=SimpleNamespace(x=0.8, y=1.2, z=1.0),
        )

    def service_is_ready(self):
        """Return the configured availability."""
        return self.available

    def call_async(self, _request):
        """Return an already-completed response future."""
        future = Future()
        future.set_result(self.response)
        return future


class FakeObstacleServiceClient(FakeServiceClient):
    """Record obstacle requests and return a successful response."""

    def __init__(self):
        super().__init__()
        self.requests = []
        self.response = SimpleNamespace(success=True, message='updated')

    def call_async(self, request):
        self.requests.append(request)
        return super().call_async(request)


class FakeThrowFoodServiceClient(FakeServiceClient):
    """Record throw-food requests and return a successful response."""

    def __init__(self):
        super().__init__()
        self.requests = []
        self.response = SimpleNamespace(success=True, message='thrown')

    def call_async(self, request):
        self.requests.append(request)
        return super().call_async(request)


class FakeFoodRequestServiceClient(FakeServiceClient):
    """Record behavior food requests and return an accepted response."""

    def __init__(self):
        super().__init__()
        self.requests = []
        self.response = SimpleNamespace(success=True, message='accepted')

    def call_async(self, request):
        self.requests.append(request)
        return super().call_async(request)


class FakeThrowFoodRequest:
    """Capture throw-food request fields set by the client."""

    def __init__(self):
        self.parking_index = 0
        self.food_name = ''
        self.x = 0.0
        self.y = 0.0
        self.orientation = [0.0, 0.0, 0.0, 0.0]


class FakeFoodRequest:
    """Capture the parking number sent to the behavior service."""

    def __init__(self):
        self.parking_number = 0


class FakeNode:
    """Provide the node operations used by ``MujocoClient``."""

    def __init__(self):
        """Create fake ROS entities."""
        self.state_client = FakeServiceClient()
        self.obstacle_client = FakeObstacleServiceClient()
        self.throw_food_client = FakeThrowFoodServiceClient()
        self.food_request_client = FakeFoodRequestServiceClient()
        self.client = self.state_client
        self.service_names = []

    def create_client(self, service_type, service_name):
        """Return the fake service client for the requested service type."""
        self.service_names.append(service_name)
        if service_type.__name__ == 'SetObstacle':
            return self.obstacle_client
        if service_type.__name__ == 'ThrowFood':
            return self.throw_food_client
        if service_type.__name__ == 'RequestFood':
            return self.food_request_client
        return self.state_client

    def create_subscription(self, *_args, **_kwargs):
        """Return a placeholder subscription for the TF listener."""
        return object()

    def destroy_subscription(self, _subscription):
        """Accept TF-listener subscription destruction."""

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


@pytest.fixture(autouse=True)
def _service_requests(monkeypatch):
    """Replace generated ROS requests with lightweight stand-ins."""
    monkeypatch.setattr(
        'simulation_interface_gui.ros.mujoco_client.ThrowFood',
        SimpleNamespace(__name__='ThrowFood', Request=FakeThrowFoodRequest),
    )
    monkeypatch.setattr(
        'simulation_interface_gui.ros.mujoco_client.RequestFood',
        SimpleNamespace(__name__='RequestFood', Request=FakeFoodRequest),
    )


def test_throw_food_maps_command_and_adds_prefix(client_and_runtime):
    """A GUI command becomes one prefixed throw-food service request."""
    client, runtime = client_and_runtime

    client.throw_food(ThrowFoodCommand(
        food_name='box',
        parking_index=2,
        x=0.25,
        y=-0.1,
        orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
    )).result()

    request = runtime.node.throw_food_client.requests[-1]
    assert request.parking_index == 2
    assert request.food_name == 'food_box'
    assert (request.x, request.y) == (0.25, -0.1)
    assert list(request.orientation) == [1.0, 0.0, 0.0, 0.0]


def test_throw_food_keeps_existing_prefix(client_and_runtime):
    """A name that already carries the prefix is not doubled."""
    client, runtime = client_and_runtime

    client.throw_food(ThrowFoodCommand(
        food_name='food_box', parking_index=1, x=0.0, y=0.0,
    )).result()

    assert runtime.node.throw_food_client.requests[-1].food_name == 'food_box'


def test_throw_food_rejects_empty_name(client_and_runtime):
    """An empty food name is rejected before any service call."""
    client, _runtime = client_and_runtime

    with pytest.raises(ValueError):
        client.throw_food(ThrowFoodCommand(
            food_name='   ', parking_index=1, x=0.0, y=0.0,
        ))


def test_throw_food_rejects_zero_orientation(client_and_runtime):
    """A degenerate zero quaternion is rejected before any service call."""
    client, _runtime = client_and_runtime

    with pytest.raises(ValueError):
        client.throw_food(ThrowFoodCommand(
            food_name='box',
            parking_index=1,
            x=0.0,
            y=0.0,
            orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
        ))


def test_throw_food_reports_unavailable_service(client_and_runtime):
    """An unavailable throw-food service completes with a useful exception."""
    client, runtime = client_and_runtime
    runtime.node.throw_food_client.available = False

    with pytest.raises(ServiceUnavailableError):
        client.throw_food(ThrowFoodCommand(
            food_name='box', parking_index=1, x=0.0, y=0.0,
        )).result()


def test_throw_food_reports_service_failure(client_and_runtime):
    """A failed throw response surfaces the plugin error message."""
    client, runtime = client_and_runtime
    runtime.node.throw_food_client.response = SimpleNamespace(
        success=False, message='no such body'
    )

    with pytest.raises(RuntimeError, match='no such body'):
        client.throw_food(ThrowFoodCommand(
            food_name='box', parking_index=1, x=0.0, y=0.0,
        )).result()


def test_request_food_sends_selected_parking(client_and_runtime):
    """A selected parking number is sent to the behavior service."""
    client, runtime = client_and_runtime

    client.request_food(3).result()

    assert runtime.node.food_request_client.requests[-1].parking_number == 3


@pytest.mark.parametrize('parking_number', [0, 5])
def test_request_food_rejects_invalid_parking(
    client_and_runtime, parking_number,
):
    """Parking numbers outside the arena range are rejected locally."""
    client, runtime = client_and_runtime

    with pytest.raises(ValueError, match='between 1 and 4'):
        client.request_food(parking_number)

    assert runtime.node.food_request_client.requests == []


def test_get_robot_state_returns_float_list(client_and_runtime):
    """The service response is exposed through immutable GUI state."""
    client, _runtime = client_and_runtime

    state = client.get_robot_state().result()

    assert state.qpos == (1.0, 2.5, -3.0)
    assert state.obstacle.position == Point3D(4.0, 5.0, 0.5)
    assert state.obstacle.length == 1.2


def test_get_sim_pose_decodes_free_joint_xy_and_yaw(client_and_runtime):
    """The simulation pose comes from the robot-state plugin qpos values."""
    client, runtime = client_and_runtime
    runtime.node.client.response.qpos = [1, 2, 0.26, 0, 0, 0, 1]

    pose = client.get_sim_pose().result()

    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 2.0, 3.141592653589793))


def test_client_uses_simulation_management_service_namespace(
    client_and_runtime,
):
    """The default services match the plugin sub-node namespace."""
    _client, runtime = client_and_runtime

    assert runtime.node.service_names == [
        '/simulation_management/get_robot_state',
        '/simulation_management/set_obstacle',
        '/simulation_management/throw_food',
        '/request_food',
    ]


def test_get_robot_state_reports_unavailable_service(client_and_runtime):
    """An unavailable state service completes with a useful exception."""
    client, runtime = client_and_runtime
    runtime.node.client.available = False

    with pytest.raises(ServiceUnavailableError):
        client.get_robot_state().result()


def test_set_obstacle_maps_position_and_dimensions(client_and_runtime):
    """Managed-box state maps to the set-obstacle service request."""
    client, runtime = client_and_runtime
    obstacle = ObstacleState(Point3D(1.0, -2.0, 0.5), 0.8, 1.2, 1.0)

    client.set_obstacle(obstacle).result()

    request = runtime.node.obstacle_client.requests[-1]
    assert (request.position.x, request.position.y, request.position.z) == (
        1.0, -2.0, 0.5,
    )
    assert (request.size.x, request.size.y, request.size.z) == (0.8, 1.2, 1.0)
