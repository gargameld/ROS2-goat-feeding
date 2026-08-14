"""Tests for GUI event forwarding."""

from concurrent.futures import Future

from simulation_interface_gui.event_listener import EventListener
from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import VelocityCommand


class FakeSignal:
    """Provide the small signal interface used by the listener."""

    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)

    def emit(self, value):
        for callback in self.callbacks:
            callback(value)


class FakeWindow:
    """Provide GUI inputs and record displayed statuses."""

    def __init__(self):
        self.velocity_command_requested = FakeSignal()
        self.obstacle_update_requested = FakeSignal()
        self.statuses = []

    def set_status(self, message, *, is_error=False):
        self.statuses.append((message, is_error))


class FakeClient:
    """Record commands forwarded to the ROS client."""

    def __init__(self):
        self.velocity_calls = []
        self.obstacle_calls = []

    def change_cmd_vel(self, **values):
        self.velocity_calls.append(values)
        future = Future()
        future.set_result(None)
        return future

    def set_obstacle(self, obstacle):
        self.obstacle_calls.append(obstacle)
        future = Future()
        future.set_result(None)
        return future


def test_listener_forwards_velocity_command():
    """A GUI velocity signal is forwarded with named ROS fields."""
    client = FakeClient()
    window = FakeWindow()
    listener = EventListener(client, window)
    listener.start()

    window.velocity_command_requested.emit(
        VelocityCommand(linear_x=1.25, linear_y=-0.5, angular_z=0.75)
    )

    assert client.velocity_calls[-1]['linear_x'] == 1.25
    assert client.velocity_calls[-1]['linear_y'] == -0.5
    assert client.velocity_calls[-1]['angular_z'] == 0.75
    assert window.statuses[-1] == ('Velocity command sent.', False)


def test_listener_forwards_obstacle_update():
    """A GUI obstacle signal is forwarded to the ROS service client."""
    client = FakeClient()
    window = FakeWindow()
    listener = EventListener(client, window)
    listener.start()
    obstacle = ObstacleState(Point3D(1.0, -2.0, 0.5), 0.8, 1.2, 1.0)

    window.obstacle_update_requested.emit(obstacle)

    assert client.obstacle_calls[-1] == obstacle
    assert window.statuses[-1] == ('Obstacle updated.', False)
