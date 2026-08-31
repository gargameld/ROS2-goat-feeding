"""Move the managed floor box with a simple arrow pad."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D


class ObstaclePanel(QGroupBox):
    """Emit box positions stepped away from the latest reported state."""

    MOVE_STEP = 0.5

    obstacle_update_requested = pyqtSignal(object)
    error_reported = pyqtSignal(str)

    _obstacle_received = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the four arrow buttons around the box position."""
        super().__init__('Obstacle box', parent)
        self._obstacle_state: ObstacleState | None = None
        layout = QVBoxLayout(self)
        arrows = QGridLayout()
        directions = (
            ('Up', 0, 1, 0.0, self.MOVE_STEP),
            ('Left', 1, 0, -self.MOVE_STEP, 0.0),
            ('Down', 1, 1, 0.0, -self.MOVE_STEP),
            ('Right', 1, 2, self.MOVE_STEP, 0.0),
        )
        for label, row, column, delta_x, delta_y in directions:
            button = QPushButton(label, self)
            button.clicked.connect(
                lambda _checked=False, x=delta_x, y=delta_y: self.move(x, y)
            )
            arrows.addWidget(button, row, column)
        layout.addLayout(arrows)
        self._obstacle_received.connect(
            self._apply_obstacle_state, Qt.QueuedConnection
        )

    def set_obstacle_state(self, obstacle: ObstacleState) -> None:
        """Queue the latest managed-box state from any thread."""
        self._obstacle_received.emit(obstacle)

    def move(self, delta_x: float, delta_y: float) -> None:
        """Request the box one step away from its last reported position."""
        current = self._obstacle_state
        if current is None:
            self.error_reported.emit('Obstacle state is not available yet.')
            return
        obstacle = ObstacleState(
            Point3D(
                current.position.x + delta_x,
                current.position.y + delta_y,
                current.position.z,
            ),
            current.width,
            current.length,
            current.height,
        )
        # Assume the request succeeds so a held arrow keeps stepping.
        self._obstacle_state = obstacle
        self.obstacle_update_requested.emit(obstacle)

    @pyqtSlot(object)
    def _apply_obstacle_state(self, obstacle: ObstacleState) -> None:
        self._obstacle_state = obstacle
