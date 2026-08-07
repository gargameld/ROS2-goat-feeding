"""Tests for complete application startup and shutdown wiring."""

from concurrent.futures import Future

import simulation_interface_gui.main as application_main


class FakeSignal:
    """Record one Qt-style signal connection."""

    def connect(self, callback):
        """Store the callback."""
        self.callback = callback


class FakeApplication:
    """Replace the Qt event loop during the startup test."""

    arguments = None

    def __init__(self, arguments):
        """Record arguments and create the quit signal."""
        type(self).arguments = arguments
        self.aboutToQuit = FakeSignal()

    def exec(self):  # noqa: A003
        """Return immediately with success."""
        return 0


class FakeRuntime:
    """Record ROS runtime lifecycle calls."""

    instance = None

    def __init__(self, ros_args):
        """Record creation."""
        self.ros_args = ros_args
        self.stopped = False
        type(self).instance = self

    def shutdown(self):
        """Record shutdown."""
        self.stopped = True


class FakeClient:
    """Record client cleanup."""

    instance = None

    def __init__(self, runtime):
        """Record the provided runtime."""
        self.runtime = runtime
        self.closed = False
        type(self).instance = self

    def close(self):
        """Return successful asynchronous cleanup."""
        self.closed = True
        future = Future()
        future.set_result(None)
        return future


class FakeWindow:
    """Record that the window was shown."""

    def show(self):
        """Record display."""
        self.shown = True


class FakeManager:
    """Record manager lifecycle calls."""

    instance = None

    def __init__(self, _client, _window):
        """Create stopped manager state."""
        self.started = False
        self.stopped = False
        type(self).instance = self

    def start(self):
        """Record startup."""
        self.started = True

    def stop(self):
        """Record shutdown."""
        self.stopped = True


def test_main_starts_and_stops_every_component(monkeypatch):
    """The entry point filters ROS args and owns every component lifecycle."""
    monkeypatch.setattr(application_main, 'QApplication', FakeApplication)
    monkeypatch.setattr(application_main, 'RosRuntime', FakeRuntime)
    monkeypatch.setattr(application_main, 'MujocoClient', FakeClient)
    monkeypatch.setattr(application_main, 'TopViewWindow', FakeWindow)
    monkeypatch.setattr(
        application_main,
        'SimulationInterfaceManager',
        FakeManager,
    )

    result = application_main.main([
        '--ros-args', '-r', '__node:=test_simulation_interface',
    ])

    assert result == 0
    assert FakeApplication.arguments == [application_main.sys.argv[0]]
    assert FakeRuntime.instance.ros_args[0] == '--ros-args'
    assert FakeRuntime.instance.stopped
    assert FakeClient.instance.closed
    assert FakeManager.instance.started
    assert FakeManager.instance.stopped
