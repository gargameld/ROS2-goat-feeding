"""Load map-specific behavior targets from YAML configuration."""

from dataclasses import dataclass
from pathlib import Path

from geometry_msgs.msg import Pose, PoseStamped
import yaml


PARKING_POSES = 'parking_poses'
HOLE_POSES = 'hole_poses'
HOLE_ARM_POSES = 'hole_arm_poses'


@dataclass(frozen=True)
class MapPose:
    """A configured pose expressed in a named map frame."""

    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    def to_pose(self) -> Pose:
        """Return this pose as an unstamped ROS pose."""
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        pose.orientation.x = self.qx
        pose.orientation.y = self.qy
        pose.orientation.z = self.qz
        pose.orientation.w = self.qw
        return pose

    def to_pose_stamped(self) -> PoseStamped:
        """Return this pose as a ROS pose stamped with its own frame."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = self.to_pose()
        return pose_stamped


class MapParametersLoader:
    """Provide validated access to map-dependent behavior parameters."""

    def __init__(self, configuration_file: str | Path):
        """Read parking and hole target poses from ``configuration_file``."""
        self.configuration_file = Path(configuration_file)
        with self.configuration_file.open(encoding='utf-8') as stream:
            configuration = yaml.safe_load(stream)

        if not isinstance(configuration, dict):
            raise ValueError('Map configuration must be a YAML mapping')

        self._poses = {
            section: self._parse_section(configuration, section)
            for section in (PARKING_POSES, HOLE_POSES, HOLE_ARM_POSES)
        }

    def get_parking_pose(self, parking_number: int) -> MapPose:
        """Return the navigation target for ``parking_number``."""
        return self._get_pose(PARKING_POSES, parking_number)

    def get_hole_pose(self, parking_number: int) -> MapPose:
        """Return the navigation target at the hole of ``parking_number``."""
        return self._get_pose(HOLE_POSES, parking_number)

    def get_hole_arm_pose(self, parking_number: int) -> MapPose:
        """Return the arm target above the hole of ``parking_number``."""
        return self._get_pose(HOLE_ARM_POSES, parking_number)

    def _get_pose(self, section: str, parking_number: int) -> MapPose:
        try:
            return self._poses[section][int(parking_number)]
        except KeyError as exc:
            raise ValueError(
                f'No {section} entry configured for parking {parking_number}'
            ) from exc

    @classmethod
    def _parse_section(cls, configuration, section) -> dict[int, MapPose]:
        poses = configuration.get(section)
        if not isinstance(poses, dict) or not poses:
            raise ValueError(f'Map configuration must define {section}')

        return {
            int(number): cls._parse_pose(section, number, pose)
            for number, pose in poses.items()
        }

    @staticmethod
    def _parse_pose(section, parking_number, configuration) -> MapPose:
        if not isinstance(configuration, dict):
            raise ValueError(
                f'{section} pose for parking {parking_number} '
                'must be a mapping'
            )

        try:
            position = configuration['position']
            orientation = configuration['orientation']
            return MapPose(
                frame_id=str(configuration.get('frame_id', 'map')),
                x=float(position['x']),
                y=float(position['y']),
                z=float(position.get('z', 0.0)),
                qx=float(orientation.get('x', 0.0)),
                qy=float(orientation.get('y', 0.0)),
                qz=float(orientation['z']),
                qw=float(orientation['w']),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f'Invalid {section} pose configured for '
                f'parking {parking_number}'
            ) from exc
