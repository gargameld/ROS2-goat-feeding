"""Tests for map-dependent behavior configuration."""

from pathlib import Path

from robot_behavior.map_parameters_loader import MapParametersLoader


def _loader():
    configuration = (
        Path(__file__).parents[1] / 'config' / 'map_parameters.yaml'
    )
    return MapParametersLoader(configuration)


def test_loads_all_parking_navigation_targets():
    """Every arena parking number has a map-frame target pose."""
    loader = _loader()

    expected_y = {1: -7.0, 2: -5.0, 3: -3.0, 4: -1.0}
    for parking_number, y in expected_y.items():
        pose = loader.get_parking_pose(parking_number)
        assert pose.frame_id == 'map'
        assert pose.x == 1.95
        assert pose.y == y
        assert pose.qz == -0.7071068
        assert pose.qw == 0.7071068


def test_loads_the_hole_targets_serving_every_parking():
    """Each parking is delivered to its own coloured hole."""
    loader = _loader()

    expected_hole_position = {
        1: (1.55, 2.1),  # blue hole
        2: (1.55, 6.1),  # red hole
        3: (-1.65, 6.1),  # black hole
        4: (-1.65, 2.1),  # magenta hole
    }
    expected_arm_position = {
        1: (2.6, 2.0, 0.75),
        2: (2.6, 6.0, 0.55),
        3: (-2.6, 6.1, 0.25),
        4: (-2.6, 2.0, 0.42),
    }
    for parking_number, (x, y) in expected_hole_position.items():
        pose = loader.get_hole_pose(parking_number)
        assert pose.frame_id == 'map'
        assert (pose.x, pose.y) == (x, y)

        arm_pose = loader.get_hole_arm_pose(parking_number)
        assert arm_pose.frame_id == 'map'
        assert (arm_pose.x, arm_pose.y, arm_pose.z) == (
            expected_arm_position[parking_number]
        )


def test_unknown_parking_number_is_rejected():
    """Only the configured parkings have delivery targets."""
    loader = _loader()

    for getter in (
        loader.get_parking_pose,
        loader.get_hole_pose,
        loader.get_hole_arm_pose,
    ):
        try:
            getter(5)
        except ValueError:
            continue
        raise AssertionError('Parking 5 should not be configured')
