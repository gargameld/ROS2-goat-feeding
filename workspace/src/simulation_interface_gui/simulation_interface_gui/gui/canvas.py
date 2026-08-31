"""Paint prepared Qt scenes for the simulation top view."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.presentation import QtScene


class TopViewCanvas(QWidget):
    """Paint a prepared Qt scene directly, without retained graphics items."""

    _scene_received = pyqtSignal(object)

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        world_bounds: tuple[float, float, float, float] = (
            -3.5, 3.5, -8.5, 8.5,
        ),
        margin: float = 12.0,
    ) -> None:
        """Create an empty canvas with fixed world bounds in metres."""
        super().__init__(parent)
        min_x, max_x, min_y, max_y = world_bounds
        if min_x >= max_x or min_y >= max_y:
            raise ValueError('World bounds must have positive width and height.')
        self._world_bounds = world_bounds
        self._margin = margin
        self._scene: QtScene | None = None
        self.setMinimumSize(360, 540)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._scene_received.connect(self._apply_scene, Qt.QueuedConnection)

    def set_scene(self, scene: QtScene) -> None:
        """Queue a scene update safely from any thread."""
        self._scene_received.emit(scene)

    @pyqtSlot(object)
    def _apply_scene(self, scene: QtScene) -> None:
        self._scene = scene
        self.update()

    def paintEvent(self, _event: object) -> None:
        """Draw the current scene through the world-to-widget transform."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(244, 246, 248))
        scene = self._scene
        if scene is None:
            painter.setPen(QColor(90, 98, 108))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for simulation state…')
            return

        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.setWorldTransform(self._world_transform())
        scene.paint(painter)

    def _world_transform(self) -> QTransform:
        """Return the metres-to-pixels transform for the current widget size."""
        min_x, max_x, min_y, max_y = self._world_bounds
        available_width = max(float(self.width()) - 2.0 * self._margin, 1.0)
        available_height = max(float(self.height()) - 2.0 * self._margin, 1.0)
        scale = min(
            available_width / (max_x - min_x),
            available_height / (max_y - min_y),
        )
        left = (float(self.width()) - (max_x - min_x) * scale) / 2.0
        top = (float(self.height()) - (max_y - min_y) * scale) / 2.0
        # World +Y points north, so the vertical axis is mirrored.
        return QTransform(
            scale, 0.0,
            0.0, -scale,
            left - min_x * scale, top + max_y * scale,
        )
