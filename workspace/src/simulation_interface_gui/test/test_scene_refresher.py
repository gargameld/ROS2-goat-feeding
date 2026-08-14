"""Tests for scene refresh scheduling."""

import pytest

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import RobotState
from simulation_interface_gui.scene_refresher import RobotStateDecoder
from simulation_interface_gui.scene_refresher import SceneRefresher


class FakeSignal:
    """Provide the signal operations used by a Qt timer."""

    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)


class FakeTimer:
    """Record timer configuration without constructing a Qt application."""

    def __init__(self):
        self.timeout = FakeSignal()
        self.interval = None
        self.running = False

    def setInterval(self, interval):
        self.interval = interval

    def start(self):
        self.running = True

    def stop(self):
        self.running = False


class FakeClient:
    """Record scene-update requests."""

    def __init__(self):
        self.state_calls = 0

    def get_robot_state(self):
        self.state_calls += 1
        raise RuntimeError('not needed for timer scheduling test')


class FakeWindow:
    """Collect errors produced by an intentionally incomplete client."""

    def __init__(self):
        self.statuses = []

    def set_status(self, message, *, is_error=False):
        self.statuses.append((message, is_error))


def test_decoder_maps_current_mjcf_qpos_layout():
    """Free-joint and arm values map into a simulation snapshot."""
    obstacle = ObstacleState(Point3D(2.0, 3.0, 0.5), 0.8, 1.2, 1.0)
    snapshot = RobotStateDecoder().decode(RobotState(tuple(range(13)), obstacle))

    assert snapshot.base_position.x == 0.0
    assert snapshot.base_position.z == 2.0
    assert snapshot.base_orientation.w == 3.0
    assert snapshot.arm_joint_positions == tuple(float(value) for value in range(7, 13))
    assert snapshot.obstacle == obstacle


def test_refresher_owns_timer_lifecycle():
    """Starting and stopping a refresher starts and stops its timer."""
    timer = FakeTimer()
    client = FakeClient()
    refresher = SceneRefresher(client, FakeWindow(), timer=timer)

    refresher.start()
    refresher.stop()

    assert timer.interval == 500
    assert client.state_calls == 1
    assert not timer.running


def test_refresher_rejects_non_positive_refresh_interval():
    """The refresher validates its own scheduling configuration."""
    with pytest.raises(ValueError, match='must be positive'):
        SceneRefresher(
            FakeClient(),
            FakeWindow(),
            refresh_interval_ms=0,
            timer=FakeTimer(),
        )
