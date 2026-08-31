"""Integration tests for the ROS executor runtime."""

import threading

from simulation_interface_gui.ros import RosRuntime


def test_runtime_executes_work_on_worker_and_shuts_down():
    """Submitted work runs on the worker and owned resources can be stopped."""
    runtime = RosRuntime(node_name='simulation_interface_runtime_test')

    try:
        worker_name = runtime.submit(
            lambda: threading.current_thread().name
        ).result(timeout=2.0)
        assert worker_name == 'simulation_interface_runtime_test-ros-executor'
    finally:
        runtime.shutdown()
