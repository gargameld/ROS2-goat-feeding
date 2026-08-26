"""Filter TCP grasp poses by their direction toward the shelf."""

import math


DEFAULT_MINIMUM_POSITIVE_X_ANGLE_DEG = 20.0
DEFAULT_MAXIMUM_POSITIVE_X_ANGLE_DEG = 110.0


def prefer_shelf_approaches(
    poses,
    minimum_positive_x_angle_deg=DEFAULT_MINIMUM_POSITIVE_X_ANGLE_DEG,
    maximum_positive_x_angle_deg=DEFAULT_MAXIMUM_POSITIVE_X_ANGLE_DEG,
):
    """Keep approaches in the requested interval from map +X toward the shelf.

    At every configured shelf parking pose the robot yaw is -90 degrees, so
    map +X is base_link +Y. The TCP's local +Z axis is its grasp approach
    direction. Candidates below ``minimum_positive_x_angle_deg`` or above
    ``maximum_positive_x_angle_deg`` from that shelf-facing axis are discarded.
    The remainder are sorted toward the shelf; top approaches remain eligible.
    """
    if not 0.0 <= minimum_positive_x_angle_deg <= 180.0:
        raise ValueError(
            'minimum_positive_x_angle_deg must be in [0, 180].'
        )
    if not (
        minimum_positive_x_angle_deg
        <= maximum_positive_x_angle_deg
        <= 180.0
    ):
        raise ValueError(
            'maximum_positive_x_angle_deg must be between the minimum and 180.'
        )

    minimum_forward_component = math.cos(
        math.radians(maximum_positive_x_angle_deg)
    )
    maximum_forward_component = math.cos(
        math.radians(minimum_positive_x_angle_deg)
    )
    eligible = [
        pose
        for pose in poses
        if minimum_forward_component
        <= _shelf_component(pose.pose.orientation)
        <= maximum_forward_component
    ]
    return sorted(
        eligible,
        key=lambda pose: _shelf_component(pose.pose.orientation),
        reverse=True,
    )


def _shelf_component(orientation):
    """Return the base +Y/map +X component of the TCP local +Z axis."""
    component = 2.0 * (
        orientation.y * orientation.z - orientation.x * orientation.w
    )
    return 0.0 if abs(component) < 1e-12 else component
