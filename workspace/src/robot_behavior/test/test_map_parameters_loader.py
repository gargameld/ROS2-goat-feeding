"""Tests for map-dependent behavior configuration."""

from pathlib import Path

from robot_behavior.map_parameters_loader import MapParametersLoader


def test_loads_all_parking_navigation_targets():
    """Every arena parking number has a map-frame target pose."""
    configuration = (
        Path(__file__).parents[1] / 'config' / 'map_parameters.yaml'
    )
    loader = MapParametersLoader(configuration)

    expected_y = {1: -7.0, 2: -5.0, 3: -3.0, 4: -1.0}
    for parking_number, y in expected_y.items():
        pose = loader.get_parking_pose(parking_number)
        assert pose.frame_id == 'map'
        assert pose.x == 1.95
        assert pose.y == y
        assert pose.qz == -0.7071068
        assert pose.qw == 0.7071068
