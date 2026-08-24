"""Data shared by states in the robot behavior state machine."""

from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Pose


@dataclass
class SharedStateData:
    """Hold values produced by one behavior state for later states."""

    grasp_pose: Optional[Pose] = None
    grasp_reference_frame: str = ''
