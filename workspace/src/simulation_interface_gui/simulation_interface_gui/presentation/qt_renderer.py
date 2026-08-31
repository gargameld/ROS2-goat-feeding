"""Translate scene states into Qt drawables for the top-view canvas."""

from abc import ABC
from abc import abstractmethod

from PyQt5.QtCore import QLineF
from PyQt5.QtCore import QPointF
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygonF

from simulation_interface_gui.presentation.qt_scene import QtDrawable
from simulation_interface_gui.presentation.qt_scene import QtLinesItem
from simulation_interface_gui.presentation.qt_scene import QtMarkersItem
from simulation_interface_gui.presentation.qt_scene import QtPolygonItem
from simulation_interface_gui.presentation.qt_scene import QtScene
from simulation_interface_gui.presentation.scene_state import Line2D
from simulation_interface_gui.presentation.scene_state import Point2D
from simulation_interface_gui.presentation.scene_state import Polygon2D
from simulation_interface_gui.presentation.scene_state import SceneState


class SceneRenderer(ABC):
    """Convert toolkit-independent scene states into Qt drawables."""

    @abstractmethod
    def render(self, scene_state: SceneState) -> QtScene:
        """Return the Qt scene that draws ``scene_state``."""


class TopViewSceneRenderer(SceneRenderer):
    """Apply the top-view palette to scene geometry, once per refresh."""

    def __init__(self) -> None:
        """Create every reusable pen and brush of the top-view palette."""
        self._shelf_pen = _pen(QColor(90, 95, 100), 1)
        self._shelf_brush = QBrush(QColor(178, 183, 188))
        self._managed_obstacle_pen = _pen(QColor(155, 70, 20), 2)
        self._managed_obstacle_brush = QBrush(QColor(235, 125, 55))
        self._boundary_pen = _pen(QColor(45, 49, 54), 2)
        self._base_pen = _pen(QColor(25, 70, 115), 2)
        self._base_brush = QBrush(QColor(83, 155, 213))
        self._orientation_pen = _pen(QColor(220, 70, 55), 3)
        self._arm_pen = _pen(QColor(230, 135, 35), 3)
        self._joint_outline_pen = _marker_pen(QColor(92, 58, 20), 9)
        self._joint_fill_pen = _marker_pen(QColor(255, 196, 76), 7)

    def render(self, scene_state: SceneState) -> QtScene:
        """Build the back-to-front drawables of one top view."""
        drawables: list[QtDrawable] = [
            QtPolygonItem(
                _polygon(shelf), self._shelf_pen, self._shelf_brush,
            )
            for shelf in scene_state.obstacle_polygons
        ]
        drawables.append(QtPolygonItem(
            _polygon(scene_state.managed_obstacle_polygon),
            self._managed_obstacle_pen,
            self._managed_obstacle_brush,
        ))
        drawables.append(QtLinesItem(
            _lines(scene_state.world_boundaries), self._boundary_pen,
        ))
        drawables.append(QtPolygonItem(
            _polygon(scene_state.base_polygon),
            self._base_pen,
            self._base_brush,
        ))
        drawables.append(QtLinesItem(
            (_line(scene_state.orientation_marker),), self._orientation_pen,
        ))
        drawables.append(QtLinesItem(
            _lines(scene_state.arm_segments), self._arm_pen,
        ))
        joints = tuple(_point(joint) for joint in scene_state.joint_markers)
        drawables.append(QtMarkersItem(joints, self._joint_outline_pen))
        drawables.append(QtMarkersItem(joints, self._joint_fill_pen))
        return QtScene(tuple(drawables))


def _pen(color: QColor, width: int) -> QPen:
    pen = QPen(color, width)
    # World coordinates are metres, so widths must not scale with the view.
    pen.setCosmetic(True)
    return pen


def _marker_pen(color: QColor, width: int) -> QPen:
    pen = _pen(color, width)
    pen.setCapStyle(Qt.RoundCap)
    return pen


def _polygon(polygon: Polygon2D) -> QPolygonF:
    return QPolygonF([_point(point) for point in polygon.points])


def _lines(segments: tuple[Line2D, ...]) -> tuple[QLineF, ...]:
    return tuple(_line(segment) for segment in segments)


def _line(segment: Line2D) -> QLineF:
    return QLineF(_point(segment.start), _point(segment.end))


def _point(point: Point2D) -> QPointF:
    return QPointF(point.x, point.y)
