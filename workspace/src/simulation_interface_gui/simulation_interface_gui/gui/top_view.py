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
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import ThrowFoodCommand
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
        painter.setPen(QPen(QColor(155, 70, 20), 2))
        painter.setBrush(QColor(235, 125, 55))
        painter.drawPolygon(self._map_polygon(scene.managed_obstacle_polygon))

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
    """Show the top view and emit validated food-throwing commands."""

    _OBSTACLE_MOVE_STEP = 0.5

    throw_food_requested = pyqtSignal(object)
    food_request_requested = pyqtSignal(int)
    obstacle_update_requested = pyqtSignal(object)
    _status_received = pyqtSignal(str, bool)
    _poses_received = pyqtSignal(object, object, object)
    _obstacle_received = pyqtSignal(object)

    _THROW_FIELDS = (
        ('Quaternion W', 'w'),
        ('Quaternion X', 'x'),
        ('Quaternion Y', 'y'),
        ('Quaternion Z', 'z'),
        ('Throw X', 'throw_x'),
        ('Throw Y', 'throw_y'),
    )

    def __init__(
        self,
        throw_food_handler: Callable[[ThrowFoodCommand], None] | None = None,
    ) -> None:
        """Create the window and optionally connect its command callback."""
        super().__init__()
        self.setWindowTitle('Simulation Interface')
        self.resize(900, 720)
        self.canvas = TopViewCanvas(self)
        self._throw_boxes: dict[str, QDoubleSpinBox] = {}
        self._food_name_edit: QLineEdit | None = None
        self._parking_box: QSpinBox | None = None
        self._request_parking_box: QSpinBox | None = None
        self._dimension_boxes: dict[str, QDoubleSpinBox] = {}
        self._obstacle_state: ObstacleState | None = None
        self._amcl_pose_label = QLabel('Waiting for map to base_link transform...')
        self._odom_pose_label = QLabel('Waiting for odom to base_link transform...')
        self._sim_pose_label = QLabel('Waiting for MuJoCo state...')
        self._status_label = QLabel('Waiting for simulation state…')
        self._status_received.connect(self._apply_status, Qt.QueuedConnection)
        self._poses_received.connect(self._apply_poses, Qt.QueuedConnection)
        self._obstacle_received.connect(
            self._apply_obstacle_state, Qt.QueuedConnection
        )
        self.setCentralWidget(self._create_content())
        if throw_food_handler is not None:
            self.throw_food_requested.connect(throw_food_handler)

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

    def set_obstacle_state(self, obstacle: ObstacleState) -> None:
        """Queue the latest managed-box state for controls and display."""
        self._obstacle_received.emit(obstacle)

    def _create_content(self) -> QWidget:
        content = QWidget(self)
        layout = QHBoxLayout(content)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self._create_controls(), 0)
        return content

    def _create_controls(self) -> QWidget:
        panel = QWidget(self)
        layout = QVBoxLayout(panel)

        throw_group = QGroupBox('Throw food', panel)
        form = QFormLayout(throw_group)
        self._food_name_edit = QLineEdit(throw_group)
        self._food_name_edit.setPlaceholderText('food object name (no prefix)')
        form.addRow('Food name', self._food_name_edit)
        self._parking_box = QSpinBox(throw_group)
        self._parking_box.setRange(1, 4)
        form.addRow('Parking number', self._parking_box)
        for label, field_name in self._THROW_FIELDS:
            spin_box = QDoubleSpinBox(throw_group)
            spin_box.setRange(-100.0, 100.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(0.1)
            if field_name.startswith('throw'):
                spin_box.setSuffix(' m')
            elif field_name == 'w':
                spin_box.setValue(1.0)
            self._throw_boxes[field_name] = spin_box
            form.addRow(label, spin_box)

        throw_button = QPushButton('Throw food', panel)
        throw_button.clicked.connect(self._send_throw_food)

        self._status_label.setWordWrap(True)
        self._status_label.setMinimumWidth(210)
        layout.addWidget(throw_group)
        layout.addWidget(throw_button)

        request_group = QGroupBox('Request food', panel)
        request_form = QFormLayout(request_group)
        self._request_parking_box = QSpinBox(request_group)
        self._request_parking_box.setRange(1, 4)
        request_form.addRow('Parking number', self._request_parking_box)
        request_button = QPushButton('Send food request', request_group)
        request_button.clicked.connect(self._send_food_request)
        request_form.addRow(request_button)
        layout.addWidget(request_group)

        obstacle_group = QGroupBox('Obstacle box', panel)
        obstacle_layout = QVBoxLayout(obstacle_group)
        dimension_form = QFormLayout()
        for label, field_name in (
            ('Height', 'height'),
            ('Width', 'width'),
            ('Length', 'length'),
        ):
            spin_box = QDoubleSpinBox(obstacle_group)
            spin_box.setRange(0.01, 10.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(0.1)
            spin_box.setSuffix(' m')
            self._dimension_boxes[field_name] = spin_box
            dimension_form.addRow(label, spin_box)
        obstacle_layout.addLayout(dimension_form)

        apply_button = QPushButton('Apply dimensions', obstacle_group)
        apply_button.clicked.connect(self._apply_obstacle_dimensions)
        obstacle_layout.addWidget(apply_button)

        arrows = QGridLayout()
        up_button = QPushButton('Up', obstacle_group)
        left_button = QPushButton('Left', obstacle_group)
        down_button = QPushButton('Down', obstacle_group)
        right_button = QPushButton('Right', obstacle_group)
        move_step = self._OBSTACLE_MOVE_STEP
        up_button.clicked.connect(lambda _checked=False: self._move_obstacle(0.0, move_step))
        left_button.clicked.connect(lambda _checked=False: self._move_obstacle(-move_step, 0.0))
        down_button.clicked.connect(lambda _checked=False: self._move_obstacle(0.0, -move_step))
        right_button.clicked.connect(lambda _checked=False: self._move_obstacle(move_step, 0.0))
        arrows.addWidget(up_button, 0, 1)
        arrows.addWidget(left_button, 1, 0)
        arrows.addWidget(down_button, 1, 1)
        arrows.addWidget(right_button, 1, 2)
        obstacle_layout.addLayout(arrows)
        layout.addWidget(obstacle_group)
        pose_group = QGroupBox('Robot pose', panel)
        pose_form = QFormLayout(pose_group)
        pose_form.addRow('AMCL map to base_link', self._amcl_pose_label)
        pose_form.addRow('EKF odom to base_link', self._odom_pose_label)
        pose_form.addRow('MuJoCo simulation', self._sim_pose_label)
        layout.addWidget(pose_group)
        layout.addStretch(1)
        layout.addWidget(self._status_label)
        return panel

    def _send_throw_food(self) -> None:
        food_name = self._food_name_edit.text().strip()
        if not food_name:
            self.set_status('Enter a food object name to throw.', is_error=True)
            return
        self.throw_food_requested.emit(ThrowFoodCommand(
            food_name=food_name,
            parking_index=self._parking_box.value(),
            x=self._throw_boxes['throw_x'].value(),
            y=self._throw_boxes['throw_y'].value(),
            orientation=Quaternion(
                w=self._throw_boxes['w'].value(),
                x=self._throw_boxes['x'].value(),
                y=self._throw_boxes['y'].value(),
                z=self._throw_boxes['z'].value(),
            ),
        ))

    def _send_food_request(self) -> None:
        """Ask the behavior node to collect food from the selected parking."""
        self.food_request_requested.emit(self._request_parking_box.value())

    def _apply_obstacle_dimensions(self) -> None:
        if self._obstacle_state is None:
            self.set_status('Obstacle state is not available yet.', is_error=True)
            return
        self._emit_obstacle_update(
            position=self._obstacle_state.position,
            width=self._dimension_boxes['width'].value(),
            length=self._dimension_boxes['length'].value(),
            height=self._dimension_boxes['height'].value(),
        )

    def _move_obstacle(self, delta_x: float, delta_y: float) -> None:
        if self._obstacle_state is None:
            self.set_status('Obstacle state is not available yet.', is_error=True)
            return
        position = self._obstacle_state.position
        self._emit_obstacle_update(
            position=Point3D(
                position.x + delta_x,
                position.y + delta_y,
                self._obstacle_state.height / 2.0,
            ),
            width=self._obstacle_state.width,
            length=self._obstacle_state.length,
            height=self._obstacle_state.height,
        )

    def _emit_obstacle_update(
        self,
        *,
        position: Point3D,
        width: float,
        length: float,
        height: float,
    ) -> None:
        obstacle = ObstacleState(position, width, length, height)
        self._obstacle_state = obstacle
        self.obstacle_update_requested.emit(obstacle)

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

    @pyqtSlot(object)
    def _apply_obstacle_state(self, obstacle: ObstacleState) -> None:
        self._obstacle_state = obstacle
        self._dimension_boxes['width'].setValue(obstacle.width)
        self._dimension_boxes['length'].setValue(obstacle.length)
        self._dimension_boxes['height'].setValue(obstacle.height)

    @staticmethod
    def _format_pose(pose: Pose2D) -> str:
        return f'x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f} rad'
