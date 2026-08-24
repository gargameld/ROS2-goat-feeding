"""Load map-specific behavior targets from YAML configuration."""

from dataclasses import dataclass
from pathlib import Path

import yaml


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


class MapParametersLoader:
    """Provide validated access to map-dependent behavior parameters."""

    def __init__(self, configuration_file: str | Path):
        """Read parking target poses from ``configuration_file``."""
        self.configuration_file = Path(configuration_file)
        with self.configuration_file.open(encoding='utf-8') as stream:
            configuration = yaml.safe_load(stream)

        if not isinstance(configuration, dict):
            raise ValueError('Map configuration must be a YAML mapping')

        parking_poses = configuration.get('parking_poses')
        if not isinstance(parking_poses, dict) or not parking_poses:
            raise ValueError('Map configuration must define parking_poses')

        self._parking_poses = {
            int(number): self._parse_pose(number, pose)
            for number, pose in parking_poses.items()
        }

    def get_parking_pose(self, parking_number: int) -> MapPose:
        """Return the navigation target for ``parking_number``."""
        try:
            return self._parking_poses[int(parking_number)]
        except KeyError as exc:
            raise ValueError(
                f'No target pose configured for parking {parking_number}'
            ) from exc

    @staticmethod
    def _parse_pose(parking_number, configuration) -> MapPose:
        if not isinstance(configuration, dict):
            raise ValueError(
                f'Pose for parking {parking_number} must be a mapping'
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
                f'Invalid pose configured for parking {parking_number}'
            ) from exc
