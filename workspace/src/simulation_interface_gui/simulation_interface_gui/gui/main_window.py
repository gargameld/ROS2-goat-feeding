"""Compose the top-view canvas and the operator controls into one window."""

from collections.abc import Callable

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.gui.canvas import TopViewCanvas
from simulation_interface_gui.gui.control_panel import ControlPanel
from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import ThrowFoodCommand
from simulation_interface_gui.presentation import PoseEstimate
from simulation_interface_gui.presentation import QtScene


class TopViewWindow(QMainWindow):
    """Hold the canvas and controls, and forward their commands and updates."""

    throw_food_requested = pyqtSignal(object)
    food_request_requested = pyqtSignal(int)
    obstacle_update_requested = pyqtSignal(object)

    def __init__(
        self,
        throw_food_handler: Callable[[ThrowFoodCommand], None] | None = None,
    ) -> None:
        """Create the window and optionally connect its command callback."""
        super().__init__()
        self.setWindowTitle('Simulation Interface')
        # Wide enough for the control panel plus a comfortable canvas.
        self.resize(1100, 720)
        self.canvas = TopViewCanvas(self)
        self.controls = ControlPanel(self)
        self.setCentralWidget(self._create_content())

        self.controls.throw_food_requested.connect(self.throw_food_requested)
        self.controls.food_request_requested.connect(
            self.food_request_requested
        )
        self.controls.obstacle_update_requested.connect(
            self.obstacle_update_requested
        )
        if throw_food_handler is not None:
            self.throw_food_requested.connect(throw_food_handler)

    def update_scene(self, scene: QtScene) -> None:
        """Queue a prepared Qt scene for display from any thread."""
        self.canvas.set_scene(scene)

    def set_status(self, message: str, *, is_error: bool = False) -> None:
        """Queue a status message from any thread."""
        self.controls.set_status(message, is_error=is_error)

    def set_poses(
        self,
        amcl_pose: PoseEstimate,
        odom_pose: PoseEstimate,
        sim_pose: PoseEstimate,
    ) -> None:
        """Queue the AMCL, odometry, and MuJoCo pose estimates for display."""
        self.controls.set_poses(amcl_pose, odom_pose, sim_pose)

    def set_obstacle_state(self, obstacle: ObstacleState) -> None:
        """Queue the latest managed-box state for controls and display."""
        self.controls.set_obstacle_state(obstacle)

    def _create_content(self) -> QWidget:
        content = QWidget(self)
        layout = QHBoxLayout(content)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.controls, 0)
        return content
