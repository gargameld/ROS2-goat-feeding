"""Low-cost Qt widgets for drawing and controlling the simulation top view."""

from collections.abc import Callable

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QPointF
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygonF
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.models import VelocityCommand
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.presentation.scene import Point2D
from simulation_interface_gui.presentation.scene import Polygon2D
from simulation_interface_gui.presentation.scene import TopViewScene


class TopViewCanvas(QWidget):
    """Paint a complete scene directly, without retained graphics items."""

    _scene_received = pyqtSignal(object)

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        world_bounds: tuple[float, float, float, float] = (
            -3.5, 3.5, -8.5, 8.5,
        ),
    ) -> None:
        """Create an empty canvas with fixed world bounds in metres."""
        super().__init__(parent)
        min_x, max_x, min_y, max_y = world_bounds
        if min_x >= max_x or min_y >= max_y:
            raise ValueError('World bounds must have positive width and height.')
        self._world_bounds = world_bounds
        self._scene: TopViewScene | None = None
        self.setMinimumSize(360, 540)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._scene_received.connect(self._apply_scene, Qt.QueuedConnection)

    def set_scene(self, scene: TopViewScene) -> None:
        """Queue a scene update safely from any thread."""
        self._scene_received.emit(scene)

    @pyqtSlot(object)
    def _apply_scene(self, scene: TopViewScene) -> None:
        self._scene = scene
        self.update()

    def paintEvent(self, _event: object) -> None:
        """Draw the current scene with basic non-antialiased primitives."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(244, 246, 248))
        scene = self._scene
        if scene is None:
            painter.setPen(QColor(90, 98, 108))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for simulation state…')
            return

        painter.setRenderHint(QPainter.Antialiasing, False)
        self._draw_obstacles(painter, scene)
        self._draw_boundaries(painter, scene)
        self._draw_robot(painter, scene)

    def _draw_obstacles(self, painter: QPainter, scene: TopViewScene) -> None:
        painter.setPen(QPen(QColor(90, 95, 100), 1))
        painter.setBrush(QColor(178, 183, 188))
        for obstacle in scene.obstacle_polygons:
            painter.drawPolygon(self._map_polygon(obstacle))

    def _draw_boundaries(self, painter: QPainter, scene: TopViewScene) -> None:
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(45, 49, 54), 2))
        for boundary in scene.world_boundaries:
            painter.drawLine(
                self._map_point(boundary.start),
                self._map_point(boundary.end),
            )

    def _draw_robot(self, painter: QPainter, scene: TopViewScene) -> None:
        painter.setPen(QPen(QColor(25, 70, 115), 2))
        painter.setBrush(QColor(83, 155, 213))
        painter.drawPolygon(self._map_polygon(scene.base_polygon))

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(220, 70, 55), 3))
        painter.drawLine(
            self._map_point(scene.orientation_marker.start),
            self._map_point(scene.orientation_marker.end),
        )

        painter.setPen(QPen(QColor(230, 135, 35), 3))
        for segment in scene.arm_segments:
            painter.drawLine(
                self._map_point(segment.start),
                self._map_point(segment.end),
            )

        painter.setPen(QPen(QColor(92, 58, 20), 1))
        painter.setBrush(QColor(255, 196, 76))
        for joint in scene.joint_markers:
            center = self._map_point(joint)
            painter.drawEllipse(center, 4.0, 4.0)

    def _map_polygon(self, polygon: Polygon2D) -> QPolygonF:
        return QPolygonF([self._map_point(point) for point in polygon.points])

    def _map_point(self, point: Point2D) -> QPointF:
        min_x, max_x, min_y, max_y = self._world_bounds
        margin = 12.0
        available_width = max(float(self.width()) - 2.0 * margin, 1.0)
        available_height = max(float(self.height()) - 2.0 * margin, 1.0)
        scale = min(
            available_width / (max_x - min_x),
            available_height / (max_y - min_y),
        )
        drawing_width = (max_x - min_x) * scale
        drawing_height = (max_y - min_y) * scale
        left = (float(self.width()) - drawing_width) / 2.0
        top = (float(self.height()) - drawing_height) / 2.0
        return QPointF(
            left + (point.x - min_x) * scale,
            top + (max_y - point.y) * scale,
        )


class TopViewWindow(QMainWindow):
    """Show the top view and emit validated velocity commands."""

    velocity_command_requested = pyqtSignal(object)
    _status_received = pyqtSignal(str, bool)
    _poses_received = pyqtSignal(object, object, object)

    _VELOCITY_FIELDS = (
        ('Linear X', 'linear_x'),
        ('Linear Y', 'linear_y'),
        ('Linear Z', 'linear_z'),
        ('Angular X', 'angular_x'),
        ('Angular Y', 'angular_y'),
        ('Angular Z', 'angular_z'),
    )

    def __init__(
        self,
        velocity_handler: Callable[[VelocityCommand], None] | None = None,
    ) -> None:
        """Create the window and optionally connect its command callback."""
        super().__init__()
        self.setWindowTitle('Simulation Interface')
        self.resize(900, 720)
        self.canvas = TopViewCanvas(self)
        self._spin_boxes: dict[str, QDoubleSpinBox] = {}
        self._amcl_pose_label = QLabel('Waiting for map to base_link transform...')
        self._odom_pose_label = QLabel('Waiting for odom to base_link transform...')
        self._sim_pose_label = QLabel('Waiting for MuJoCo state...')
        self._status_label = QLabel('Waiting for simulation state…')
        self._status_received.connect(self._apply_status, Qt.QueuedConnection)
        self._poses_received.connect(self._apply_poses, Qt.QueuedConnection)
        self.setCentralWidget(self._create_content())
        if velocity_handler is not None:
            self.velocity_command_requested.connect(velocity_handler)

    def update_scene(self, scene: TopViewScene) -> None:
        """Queue a scene for display from any thread."""
        self.canvas.set_scene(scene)

    def set_status(self, message: str, *, is_error: bool = False) -> None:
        """Queue a status message from any thread."""
        self._status_received.emit(message, is_error)

    def set_poses(
        self, amcl_pose: Pose2D, odom_pose: Pose2D, sim_pose: Pose2D,
    ) -> None:
        """Queue the AMCL, odometry, and MuJoCo poses for display."""
        self._poses_received.emit(amcl_pose, odom_pose, sim_pose)

    def _create_content(self) -> QWidget:
        content = QWidget(self)
        layout = QHBoxLayout(content)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self._create_controls(), 0)
        return content

    def _create_controls(self) -> QWidget:
        panel = QWidget(self)
        layout = QVBoxLayout(panel)

        command_group = QGroupBox('Velocity command', panel)
        form = QFormLayout(command_group)
        for label, field_name in self._VELOCITY_FIELDS:
            spin_box = QDoubleSpinBox(command_group)
            spin_box.setRange(-10.0, 10.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(0.1)
            spin_box.setSuffix(
                ' m/s' if field_name.startswith('linear') else ' rad/s'
            )
            self._spin_boxes[field_name] = spin_box
            form.addRow(label, spin_box)

        send_button = QPushButton('Send command', panel)
        send_button.clicked.connect(self._send_velocity)
        stop_button = QPushButton('Stop robot', panel)
        stop_button.clicked.connect(self._send_stop)

        self._status_label.setWordWrap(True)
        self._status_label.setMinimumWidth(210)
        layout.addWidget(command_group)
        layout.addWidget(send_button)
        layout.addWidget(stop_button)
        pose_group = QGroupBox('Robot pose', panel)
        pose_form = QFormLayout(pose_group)
        pose_form.addRow('AMCL map to base_link', self._amcl_pose_label)
        pose_form.addRow('EKF odom to base_link', self._odom_pose_label)
        pose_form.addRow('MuJoCo simulation', self._sim_pose_label)
        layout.addWidget(pose_group)
        layout.addStretch(1)
        layout.addWidget(self._status_label)
        return panel

    def _send_velocity(self) -> None:
        values = {
            field_name: spin_box.value()
            for field_name, spin_box in self._spin_boxes.items()
        }
        self.velocity_command_requested.emit(VelocityCommand(**values))

    def _send_stop(self) -> None:
        for spin_box in self._spin_boxes.values():
            spin_box.setValue(0.0)
        self.velocity_command_requested.emit(VelocityCommand())

    @pyqtSlot(str, bool)
    def _apply_status(self, message: str, is_error: bool) -> None:
        color = '#a12622' if is_error else '#276738'
        self._status_label.setStyleSheet(f'color: {color};')
        self._status_label.setText(message)

    @pyqtSlot(object, object, object)
    def _apply_poses(
        self, amcl_pose: Pose2D, odom_pose: Pose2D, sim_pose: Pose2D,
    ) -> None:
        self._amcl_pose_label.setText(self._format_pose(amcl_pose))
        self._odom_pose_label.setText(self._format_pose(odom_pose))
        self._sim_pose_label.setText(self._format_pose(sim_pose))

    @staticmethod
    def _format_pose(pose: Pose2D) -> str:
        return f'x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f} rad'
