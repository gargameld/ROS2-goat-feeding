"""Application entry point for the simulation interface GUI."""

import sys

from PyQt5.QtWidgets import QApplication
from rclpy.utilities import remove_ros_args

from simulation_interface_gui.event_listener import EventListener
from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.presentation import MujocoSimulationStateProvider
from simulation_interface_gui.presentation import TopViewSceneComposer
from simulation_interface_gui.ros import MujocoClient
from simulation_interface_gui.ros import RosRuntime
from simulation_interface_gui.scene_refresher import SceneRefresher


def main(args: list[str] | None = None) -> int:
    """Create all application components and run the Qt event loop."""
    complete_arguments = list(sys.argv) if args is None else [sys.argv[0], *args]
    runtime = RosRuntime(ros_args=args)
    client = None
    event_listener = None
    scene_refresher = None

    try:
        application = QApplication(remove_ros_args(args=complete_arguments))
        client = MujocoClient(runtime)
        window = TopViewWindow()
        event_listener = EventListener(client, window)
        scene_refresher = SceneRefresher(
            TopViewSceneComposer(MujocoSimulationStateProvider(client)),
            window,
        )
        application.aboutToQuit.connect(event_listener.stop)
        application.aboutToQuit.connect(scene_refresher.stop)

        window.show()
        event_listener.start()
        scene_refresher.start()
        return application.exec()
    finally:
        if event_listener is not None:
            event_listener.stop()
        if scene_refresher is not None:
            scene_refresher.stop()
        try:
            if client is not None:
                client.close().result(timeout=2.0)
        finally:
            runtime.shutdown()
