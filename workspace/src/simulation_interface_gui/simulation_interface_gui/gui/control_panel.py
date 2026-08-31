"""Assemble every operator control into one side panel."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.gui.food_request_panel import FoodRequestPanel
from simulation_interface_gui.gui.obstacle_panel import ObstaclePanel
from simulation_interface_gui.gui.pose_panel import PosePanel
from simulation_interface_gui.gui.status_view import StatusView
from simulation_interface_gui.gui.throw_food_panel import ThrowFoodPanel
from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.presentation import PoseEstimate


class ControlPanel(QWidget):
    """Lay out the control panels and route their reports to the status view."""

    throw_food_requested = pyqtSignal(object)
    food_request_requested = pyqtSignal(int)
    obstacle_update_requested = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create every control panel and connect it to this panel's signals."""
        super().__init__(parent)
        self.throw_food_panel = ThrowFoodPanel(self)
        self.food_request_panel = FoodRequestPanel(self)
        self.obstacle_panel = ObstaclePanel(self)
        self.pose_panel = PosePanel(self)
        self.status_view = StatusView(self)

        layout = QVBoxLayout(self)
        layout.addWidget(self.throw_food_panel)
        layout.addWidget(self.food_request_panel)
        layout.addWidget(self.obstacle_panel)
        layout.addWidget(self.pose_panel)
        layout.addStretch(1)
        layout.addWidget(self.status_view)

        self.throw_food_panel.throw_requested.connect(self.throw_food_requested)
        self.food_request_panel.food_requested.connect(
            self.food_request_requested
        )
        self.obstacle_panel.obstacle_update_requested.connect(
            self.obstacle_update_requested
        )
        # Freeze the panel at the width its controls need, so that no
        # message can widen it and shrink the canvas.
        self.setMaximumWidth(self.sizeHint().width())

        self.throw_food_panel.error_reported.connect(self._report_error)
        self.obstacle_panel.error_reported.connect(self._report_error)

    def set_status(self, message: str, *, is_error: bool = False) -> None:
        """Queue a status message from any thread."""
        self.status_view.set_status(message, is_error=is_error)

    def set_poses(
        self,
        amcl_pose: PoseEstimate,
        odom_pose: PoseEstimate,
        sim_pose: PoseEstimate,
    ) -> None:
        """Queue every pose estimate for display from any thread."""
        self.pose_panel.set_poses(amcl_pose, odom_pose, sim_pose)

    def set_obstacle_state(self, obstacle: ObstacleState) -> None:
        """Queue the latest managed-box state from any thread."""
        self.obstacle_panel.set_obstacle_state(obstacle)

    def _report_error(self, message: str) -> None:
        self.status_view.set_status(message, is_error=True)
