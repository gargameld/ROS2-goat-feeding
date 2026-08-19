"""Tests for GUI event forwarding."""

from concurrent.futures import Future

from simulation_interface_gui.event_listener import EventListener
from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import ThrowFoodCommand


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
        self.throw_food_requested = FakeSignal()
        self.obstacle_update_requested = FakeSignal()
        self.statuses = []

    def set_status(self, message, *, is_error=False):
        self.statuses.append((message, is_error))


class FakeClient:
    """Record commands forwarded to the ROS client."""

    def __init__(self):
        self.throw_calls = []
        self.obstacle_calls = []

    def throw_food(self, command):
        self.throw_calls.append(command)
        future = Future()
        future.set_result(None)
        return future

    def set_obstacle(self, obstacle):
        self.obstacle_calls.append(obstacle)
        future = Future()
        future.set_result(None)
        return future


def test_listener_forwards_throw_food_command():
    """A GUI throw-food signal is forwarded to the ROS client."""
    client = FakeClient()
    window = FakeWindow()
    listener = EventListener(client, window)
    listener.start()
    command = ThrowFoodCommand(
        food_name='box',
        parking_index=2,
        x=0.25,
        y=-0.1,
        orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
    )

    window.throw_food_requested.emit(command)

    assert client.throw_calls[-1] == command
    assert window.statuses[-1] == ('Food thrown.', False)


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
