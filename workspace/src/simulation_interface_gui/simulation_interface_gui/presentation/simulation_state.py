"""Immutable description of one sample of the running simulation."""

from dataclasses import dataclass
from dataclasses import field

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import Quaternion


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    """Contain one pose source that may be temporarily unavailable."""

    pose: Pose2D | None = None
    error: str | None = None

    @classmethod
    def of(cls, pose: Pose2D) -> 'PoseEstimate':
        """Return an available estimate holding one measured pose."""
        return cls(pose=pose)

    @classmethod
    def unavailable(cls, reason: str) -> 'PoseEstimate':
        """Return an unavailable estimate explaining the missing pose."""
        return cls(error=reason)


@dataclass(frozen=True, slots=True)
class SimulationState:
    """Contain every simulation value the interface presents in one refresh."""

    base_position: Point3D
    base_orientation: Quaternion
    arm_points_world: tuple[Point3D, ...]
    obstacle: ObstacleState
    amcl_pose: PoseEstimate = field(default_factory=PoseEstimate)
    odom_pose: PoseEstimate = field(default_factory=PoseEstimate)
    sim_pose: PoseEstimate = field(default_factory=PoseEstimate)
