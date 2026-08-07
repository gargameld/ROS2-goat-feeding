"""Translate simulation snapshots into lightweight top-view scenes."""

import math

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import SimulationSnapshot
from simulation_interface_gui.presentation.kinematics import ArmKinematics
from simulation_interface_gui.presentation.kinematics import quaternion_to_yaw
from simulation_interface_gui.presentation.kinematics import robot_point_to_world
from simulation_interface_gui.presentation.scene import Line2D
from simulation_interface_gui.presentation.scene import Point2D
from simulation_interface_gui.presentation.scene import Polygon2D
from simulation_interface_gui.presentation.scene import TopViewScene


_BASE_CORNERS = (
    Point3D(-0.40, -0.50, 0.0),
    Point3D(0.40, -0.50, 0.0),
    Point3D(0.40, 0.50, 0.0),
    Point3D(-0.40, 0.50, 0.0),
)
_ORIENTATION_POINT = Point3D(0.0, 0.65, 0.0)


def _line(start_x: float, start_y: float, end_x: float, end_y: float) -> Line2D:
    return Line2D(Point2D(start_x, start_y), Point2D(end_x, end_y))


def _rectangle(
    center_x: float,
    center_y: float,
    half_width: float,
    half_height: float,
) -> Polygon2D:
    return Polygon2D((
        Point2D(center_x - half_width, center_y - half_height),
        Point2D(center_x + half_width, center_y - half_height),
        Point2D(center_x + half_width, center_y + half_height),
        Point2D(center_x - half_width, center_y + half_height),
    ))


# Static centre-lines derived from the box geoms in scene.xml. Thin walls are
# lines and only shelf footprints are polygons, keeping each refresh cheap.
_ARENA_BOUNDARIES = (
    _line(-2.5, -8.0, -2.5, 8.0),
    _line(-2.5, 8.0, 2.5, 8.0),
    _line(2.5, 0.0, 2.5, 8.0),
    _line(-2.5, -8.0, 2.5, -8.0),
)
_PARKING_CENTERS = (-7.0, -5.0, -3.0, -1.0)
_PARKING_BOUNDARIES = tuple(
    boundary
    for center_y in _PARKING_CENTERS
    for boundary in (
        _line(2.45, center_y - 0.7, 3.05, center_y - 0.7),
        _line(2.45, center_y + 0.7, 3.05, center_y + 0.7),
        _line(3.0, center_y - 0.7, 3.0, center_y + 0.7),
        _line(2.5, center_y - 1.0, 2.5, center_y - 0.7),
        _line(2.5, center_y + 0.7, 2.5, center_y + 1.0),
    )
)
DEFAULT_WORLD_BOUNDARIES = _ARENA_BOUNDARIES + _PARKING_BOUNDARIES
DEFAULT_OBSTACLES = tuple(
    _rectangle(2.75, center_y, 0.25, 0.8)
    for center_y in _PARKING_CENTERS
)


class SceneBuilder:
    """Build complete immutable top-view scenes without ROS or Qt types."""

    def __init__(
        self,
        *,
        arm_kinematics: ArmKinematics | None = None,
        world_boundaries: tuple[Line2D, ...] = DEFAULT_WORLD_BOUNDARIES,
        obstacle_polygons: tuple[Polygon2D, ...] = DEFAULT_OBSTACLES,
    ) -> None:
        """Configure robot kinematics and static world geometry."""
        self._arm_kinematics = arm_kinematics or ArmKinematics()
        self._world_boundaries = world_boundaries
        self._obstacle_polygons = obstacle_polygons

    def build(self, snapshot: SimulationSnapshot) -> TopViewScene:
        """Translate one simulation snapshot into a complete top-view scene."""
        yaw = quaternion_to_yaw(snapshot.base_orientation)
        center = _project(snapshot.base_position)

        base_polygon = Polygon2D(tuple(
            _project(robot_point_to_world(corner, snapshot.base_position, yaw))
            for corner in _BASE_CORNERS
        ))
        orientation_marker = Line2D(
            center,
            _project(robot_point_to_world(
                _ORIENTATION_POINT,
                snapshot.base_position,
                yaw,
            )),
        )

        relative_arm_points = self._arm_kinematics.forward(
            snapshot.arm_joint_positions
        )
        arm_points = tuple(
            _project(robot_point_to_world(point, snapshot.base_position, yaw))
            for point in relative_arm_points
        )
        arm_segments = tuple(
            Line2D(start, end)
            for start, end in zip(arm_points, arm_points[1:])
        )

        return TopViewScene(
            base_polygon=base_polygon,
            orientation_marker=orientation_marker,
            arm_segments=arm_segments,
            joint_markers=arm_points[:-1],
            world_boundaries=self._world_boundaries,
            obstacle_polygons=self._obstacle_polygons,
        )


def _project(point: Point3D) -> Point2D:
    if not math.isfinite(point.x) or not math.isfinite(point.y):
        raise ValueError('Scene coordinates must be finite.')
    return Point2D(point.x, point.y)
