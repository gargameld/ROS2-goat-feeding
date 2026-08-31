"""Immutable, toolkit-independent geometry of one drawable scene."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Point2D:
    """Represent a point in world-space metres."""

    x: float
    y: float


@dataclass(frozen=True, slots=True)
class Line2D:
    """Represent a line segment between two world-space points."""

    start: Point2D
    end: Point2D


@dataclass(frozen=True, slots=True)
class Polygon2D:
    """Represent a closed polygon whose final edge returns to its first point."""

    points: tuple[Point2D, ...]

    def __post_init__(self) -> None:
        """Reject shapes that cannot form a polygon."""
        if len(self.points) < 3:
            raise ValueError('A polygon requires at least three points.')


@dataclass(frozen=True, slots=True)
class SceneState:
    """Contain every line and polygon needed to draw one top view."""

    base_polygon: Polygon2D
    orientation_marker: Line2D
    arm_segments: tuple[Line2D, ...]
    joint_markers: tuple[Point2D, ...]
    world_boundaries: tuple[Line2D, ...]
    obstacle_polygons: tuple[Polygon2D, ...]
    managed_obstacle_polygon: Polygon2D
