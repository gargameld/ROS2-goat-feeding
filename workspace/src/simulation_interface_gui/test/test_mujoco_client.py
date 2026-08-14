"""Unit tests for the GUI-facing MuJoCo ROS client."""

from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
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


class FakeNode:
    """Provide the node operations used by ``MujocoClient``."""

    def __init__(self):
        """Create fake ROS entities."""
        self.publisher = FakePublisher()
        self.state_client = FakeServiceClient()
        self.obstacle_client = FakeObstacleServiceClient()
        self.client = self.state_client
        self.publisher_message_type = None
        self.publisher_topic = None
        self.timer_callback = None
        self.timer_period = None
        self.timer_destroyed = False
        self.service_names = []
        self.stamp = SimpleNamespace(sec=12, nanosec=34)

    def create_publisher(self, message_type, topic, _depth):
        """Return the fake publisher."""
        self.publisher_message_type = message_type
        self.publisher_topic = topic
        return self.publisher

    def get_clock(self):
        """Return a clock that provides a deterministic ROS timestamp."""
        time = SimpleNamespace(to_msg=lambda: self.stamp)
        return SimpleNamespace(now=lambda: time)

    def create_timer(self, period, callback):
        """Record the periodic command publisher."""
        self.timer_period = period
        self.timer_callback = callback
        return callback

    def destroy_timer(self, _timer):
        """Record timer destruction."""
        self.timer_destroyed = True

    def create_client(self, service_type, service_name):
        """Return the fake service client."""
        self.service_names.append(service_name)
        if service_type.__name__ == 'SetObstacle':
            return self.obstacle_client
        return self.state_client

    def create_subscription(self, *_args, **_kwargs):
        """Return a placeholder subscription for the TF listener."""
        return object()

    def destroy_subscription(self, _subscription):
        """Accept TF-listener subscription destruction."""

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
    """The six GUI velocity fields are mapped to one stamped reference."""
    client, runtime = client_and_runtime

    client.change_cmd_vel(1, 2, 3, 4, 5, 6).result()

    message = runtime.node.publisher.messages[-1]
    assert message.header.stamp is runtime.node.stamp
    assert (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
    ) == (1, 2, 3)
    assert (
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    ) == (4, 5, 6)


def test_client_uses_mecanum_controller_reference_topic(client_and_runtime):
    """The default publisher matches the Jazzy mecanum command interface."""
    _client, runtime = client_and_runtime

    assert runtime.node.publisher_message_type.__name__ == 'TwistStamped'
    assert runtime.node.publisher_topic == '/mecanum_drive_controller/reference'


def test_velocity_command_is_republished_for_the_controller(client_and_runtime):
    """A selected command remains available across controller updates."""
    client, runtime = client_and_runtime
    client.change_cmd_vel(linear_x=1.5).result()

    runtime.node.timer_callback()

    assert runtime.node.timer_period == pytest.approx(0.05)
    assert len(runtime.node.publisher.messages) == 2
    assert runtime.node.publisher.messages[-1].twist.linear.x == 1.5


def test_close_publishes_stop_and_destroys_command_timer(client_and_runtime):
    """Closing the GUI leaves the velocity controller with a stop command."""
    client, runtime = client_and_runtime
    client.change_cmd_vel(linear_x=1.0).result()

    client.close().result()

    assert runtime.node.publisher.messages[-1].twist.linear.x == 0.0
    assert runtime.node.timer_destroyed


def test_change_cmd_vel_rejects_non_finite_values(client_and_runtime):
    """Invalid textbox values are rejected before publishing."""
    client, _runtime = client_and_runtime

    with pytest.raises(ValueError):
        client.change_cmd_vel(linear_x=float('nan'))


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
    """The default service matches the plugin sub-node namespace."""
    _client, runtime = client_and_runtime

    assert runtime.node.service_names == [
        '/simulation_management/get_robot_state',
        '/simulation_management/set_obstacle',
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
