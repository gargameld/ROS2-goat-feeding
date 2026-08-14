"""Immutable simulation state consumed by the presentation layer."""

from dataclasses import dataclass
from dataclasses import field


@dataclass(frozen=True, slots=True)
class Pose2D:
    """Represent a planar pose in metres and radians."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True, slots=True)
class Point3D:
    """Represent a position in three-dimensional world or robot coordinates."""

    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Quaternion:
    """Represent an orientation quaternion in MuJoCo's ``w, x, y, z`` order."""

    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class ObstacleState:
    """Represent the managed floor box using full dimensions in metres."""

    position: Point3D
    width: float
    length: float
    height: float


@dataclass(frozen=True, slots=True)
class RobotState:
    """Contain one service response from the simulation-management plugin."""

    qpos: tuple[float, ...]
    obstacle: ObstacleState


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    """Contain the state needed to construct one top-view scene."""

    base_position: Point3D
    base_orientation: Quaternion
    arm_joint_positions: tuple[float, ...]
    obstacle: ObstacleState = field(default_factory=lambda: ObstacleState(
        position=Point3D(-2.0, -7.5, 0.5),
        width=0.8,
        length=0.8,
        height=1.0,
    ))


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    """Contain the six components of a ROS ``Twist`` command."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0
