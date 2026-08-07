"""Immutable, toolkit-independent descriptions of a top-view scene."""

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
class Circle2D:
    """Represent a circle in world-space metres."""

    center: Point2D
    radius: float

    def __post_init__(self) -> None:
        """Reject a negative radius."""
        if self.radius < 0.0:
            raise ValueError('A circle radius cannot be negative.')


@dataclass(frozen=True, slots=True)
class TopViewScene:
    """Contain every primitive needed to draw one lightweight top view."""

    base_polygon: Polygon2D
    orientation_marker: Line2D
    arm_segments: tuple[Line2D, ...]
    joint_markers: tuple[Point2D, ...]
    world_boundaries: tuple[Line2D, ...]
    obstacle_polygons: tuple[Polygon2D, ...]
