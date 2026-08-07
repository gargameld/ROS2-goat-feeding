"""Application entry point for the simulation interface GUI."""

import sys

from PyQt5.QtWidgets import QApplication
from rclpy.utilities import remove_ros_args

from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.manager import SimulationInterfaceManager
from simulation_interface_gui.ros import MujocoClient
from simulation_interface_gui.ros import RosRuntime


def main(args: list[str] | None = None) -> int:
    """Create all application components and run the Qt event loop."""
    complete_arguments = list(sys.argv) if args is None else [sys.argv[0], *args]
    runtime = RosRuntime(ros_args=args)
    client = None
    manager = None

    try:
        application = QApplication(remove_ros_args(args=complete_arguments))
        client = MujocoClient(runtime)
        window = TopViewWindow()
        manager = SimulationInterfaceManager(client, window)
        application.aboutToQuit.connect(manager.stop)

        window.show()
        manager.start()
        return application.exec()
    finally:
        if manager is not None:
            manager.stop()
        try:
            if client is not None:
                client.close().result(timeout=2.0)
        finally:
            runtime.shutdown()
