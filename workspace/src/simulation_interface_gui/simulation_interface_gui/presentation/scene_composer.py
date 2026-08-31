"""Turn simulation states into drawable scene states."""

from abc import ABC
from abc import abstractmethod
from concurrent.futures import Future
from dataclasses import dataclass
import math

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.presentation.kinematics import quaternion_to_yaw
from simulation_interface_gui.presentation.kinematics import robot_point_to_world
from simulation_interface_gui.presentation.scene_state import Line2D
from simulation_interface_gui.presentation.scene_state import Point2D
from simulation_interface_gui.presentation.scene_state import Polygon2D
from simulation_interface_gui.presentation.scene_state import SceneState
from simulation_interface_gui.presentation.simulation_state import SimulationState
from simulation_interface_gui.presentation.simulation_state_provider import (
    SimulationStateProvider,
)


_BASE_CORNERS = (
    Point3D(-0.50, -0.40, 0.0),
    Point3D(0.50, -0.40, 0.0),
    Point3D(0.50, 0.40, 0.0),
    Point3D(-0.50, 0.40, 0.0),
)
# robot.xml defines chassis +X as forward and +Y as left.
_ORIENTATION_POINT = Point3D(0.65, 0.0, 0.0)


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


@dataclass(frozen=True, slots=True)
class SceneUpdate:
    """Contain one simulation state and the scene state composed from it."""

    simulation_state: SimulationState
    scene_state: SceneState


class SceneComposer(ABC):
    """Compose scene states, both on demand and from the simulation."""

    @abstractmethod
    def compose(self, simulation_state: SimulationState) -> SceneState:
        """Translate one simulation state into drawable scene geometry."""

    @abstractmethod
    def request_scene_update(self) -> Future[SceneUpdate]:
        """Update the simulation state and compose its scene state."""


class TopViewSceneComposer(SceneComposer):
    """Compose top-view scenes from states read by a lower-level provider."""

    def __init__(
        self,
        provider: SimulationStateProvider,
        *,
        world_boundaries: tuple[Line2D, ...] = DEFAULT_WORLD_BOUNDARIES,
        obstacle_polygons: tuple[Polygon2D, ...] = DEFAULT_OBSTACLES,
    ) -> None:
        """Store the state provider and this scene's static world geometry."""
        self._provider = provider
        self._world_boundaries = world_boundaries
        self._obstacle_polygons = obstacle_polygons

    def request_scene_update(self) -> Future[SceneUpdate]:
        """Ask the provider for a state and compose its scene when it lands."""
        result: Future[SceneUpdate] = Future()
        result.set_running_or_notify_cancel()

        def finish(state_future: Future[SimulationState]) -> None:
            try:
                simulation_state = state_future.result()
                result.set_result(SceneUpdate(
                    simulation_state=simulation_state,
                    scene_state=self.compose(simulation_state),
                ))
            except BaseException as error:
                result.set_exception(error)

        try:
            self._provider.request_simulation_state().add_done_callback(finish)
        except BaseException as error:
            result.set_exception(error)
        return result

    def compose(self, simulation_state: SimulationState) -> SceneState:
        """Translate one simulation state into a complete top-view scene."""
        yaw = quaternion_to_yaw(simulation_state.base_orientation)
        center = _project(simulation_state.base_position)

        base_polygon = Polygon2D(tuple(
            _project(robot_point_to_world(
                corner, simulation_state.base_position, yaw
            ))
            for corner in _BASE_CORNERS
        ))
        orientation_marker = Line2D(
            center,
            _project(robot_point_to_world(
                _ORIENTATION_POINT,
                simulation_state.base_position,
                yaw,
            )),
        )

        arm_points = tuple(
            _project(point) for point in simulation_state.arm_points_world
        )
        arm_segments = tuple(
            Line2D(start, end)
            for start, end in zip(arm_points, arm_points[1:])
        )

        return SceneState(
            base_polygon=base_polygon,
            orientation_marker=orientation_marker,
            arm_segments=arm_segments,
            joint_markers=arm_points[:-1],
            world_boundaries=self._world_boundaries,
            obstacle_polygons=self._obstacle_polygons,
            managed_obstacle_polygon=_rectangle(
                simulation_state.obstacle.position.x,
                simulation_state.obstacle.position.y,
                simulation_state.obstacle.width / 2.0,
                simulation_state.obstacle.length / 2.0,
            ),
        )


def _project(point: Point3D) -> Point2D:
    if not math.isfinite(point.x) or not math.isfinite(point.y):
        raise ValueError('Scene coordinates must be finite.')
    return Point2D(point.x, point.y)
