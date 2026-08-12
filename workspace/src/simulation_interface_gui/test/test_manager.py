"""Tests for ROS-to-presentation manager coordination."""

from concurrent.futures import Future

import pytest

from simulation_interface_gui.manager import RobotStateDecoder
from simulation_interface_gui.manager import SimulationInterfaceManager
from simulation_interface_gui.models import VelocityCommand
from simulation_interface_gui.models import Pose2D


class FakeSignal:
    """Provide the small signal interface used by the manager."""

    def __init__(self):
        """Create an empty callback list."""
        self.callbacks = []

    def connect(self, callback):
        """Register a callback."""
        self.callbacks.append(callback)

    def emit(self, value=None):
        """Invoke registered callbacks."""
        for callback in self.callbacks:
            callback() if value is None else callback(value)


class FakeTimer:
    """Allow deterministic timer triggering."""

    def __init__(self):
        """Create a stopped timer."""
        self.timeout = FakeSignal()
        self.interval = None
        self.running = False

    def setInterval(self, interval):
        """Record the configured interval."""
        self.interval = interval

    def start(self):
        """Mark the timer as running."""
        self.running = True

    def stop(self):
        """Mark the timer as stopped."""
        self.running = False

    def trigger(self):
        """Emit one timeout."""
        self.timeout.emit()


class FakeWindow:
    """Collect manager outputs without constructing Qt widgets."""

    def __init__(self):
        """Create empty output collections."""
        self.velocity_command_requested = FakeSignal()
        self.scenes = []
        self.poses = []
        self.statuses = []

    def update_scene(self, scene):
        """Record one scene."""
        self.scenes.append(scene)

    def set_status(self, message, *, is_error=False):
        """Record one status update."""
        self.statuses.append((message, is_error))

    def set_poses(self, amcl_pose, odom_pose, sim_pose):
        """Record one displayed triple of poses."""
        self.poses.append((amcl_pose, odom_pose, sim_pose))


class FakeClient:
    """Provide configurable asynchronous ROS operations."""

    def __init__(self, state_future=None):
        """Create a client returning a valid default qpos state."""
        self.state_future = state_future
        self.state_calls = 0
        self.velocity_calls = []

    def get_robot_state(self):
        """Return a configured or immediate state future."""
        self.state_calls += 1
        if self.state_future is not None:
            return self.state_future
        future = Future()
        future.set_result([1.0, 2.0, 0.26, 1.0, 0.0, 0.0, 0.0] + [0.0] * 6)
        return future

    def change_cmd_vel(self, **values):
        """Record velocity fields and return successful completion."""
        self.velocity_calls.append(values)
        future = Future()
        future.set_result(None)
        return future

    @staticmethod
    def get_amcl_pose():
        """Return a representative map-frame robot pose."""
        future = Future()
        future.set_result(Pose2D(3.0, 4.0, 0.5))
        return future

    @staticmethod
    def get_sim_pose():
        """Return a representative MuJoCo robot pose."""
        future = Future()
        future.set_result(Pose2D(1.0, 2.0, 0.25))
        return future

    @staticmethod
    def get_odom_pose():
        """Return a representative EKF odometry pose."""
        future = Future()
        future.set_result(Pose2D(2.0, 3.0, 0.4))
        return future


def test_decoder_maps_current_mjcf_qpos_layout():
    """Free-joint and arm values are mapped into a simulation snapshot."""
    snapshot = RobotStateDecoder().decode(tuple(range(13)))

    assert snapshot.base_position.x == 0.0
    assert snapshot.base_position.z == 2.0
    assert snapshot.base_orientation.w == 3.0
    assert snapshot.arm_joint_positions == tuple(float(value) for value in range(7, 13))


def test_manager_builds_scene_on_start():
    """Starting the manager requests state and delivers a complete scene."""
    client = FakeClient()
    window = FakeWindow()
    timer = FakeTimer()
    manager = SimulationInterfaceManager(client, window, timer=timer)

    manager.start()

    assert timer.interval == 500
    assert timer.running
    assert client.state_calls == 1
    assert len(window.scenes) == 1
    assert window.scenes[0].orientation_marker.start.x == pytest.approx(1.0)
    assert window.scenes[0].orientation_marker.start.y == pytest.approx(2.0)
    assert window.poses[-1] == (
        Pose2D(3.0, 4.0, 0.5),
        Pose2D(2.0, 3.0, 0.4),
        Pose2D(1.0, 2.0, 0.25),
    )


def test_manager_does_not_overlap_state_requests():
    """Timer ticks are skipped while the previous service call is pending."""
    pending = Future()
    client = FakeClient(pending)
    window = FakeWindow()
    timer = FakeTimer()
    manager = SimulationInterfaceManager(client, window, timer=timer)
    manager.start()

    timer.trigger()

    assert client.state_calls == 1


def test_manager_forwards_velocity_callback():
    """GUI commands are forwarded to ``MujocoClient`` with named fields."""
    client = FakeClient()
    window = FakeWindow()
    manager = SimulationInterfaceManager(client, window, timer=FakeTimer())
    manager.start()
    command = VelocityCommand(linear_x=1.25, linear_y=-0.5, angular_z=0.75)

    window.velocity_command_requested.emit(command)

    assert client.velocity_calls[-1]['linear_x'] == 1.25
    assert client.velocity_calls[-1]['linear_y'] == -0.5
    assert client.velocity_calls[-1]['angular_z'] == 0.75
