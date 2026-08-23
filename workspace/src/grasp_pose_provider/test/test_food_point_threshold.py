from grasp_pose_provider import food_presence
import pytest


def test_food_point_threshold_accepts_minimum_count():
    food_presence.require_minimum_food_points(range(10), 10)


@pytest.mark.parametrize('point_count', [0, 1, 9])
def test_food_point_threshold_rejects_empty_or_tiny_cloud(point_count):
    with pytest.raises(food_presence.NoFoodDetectedError) as exc_info:
        food_presence.require_minimum_food_points(
            range(point_count),
            10,
        )

    assert exc_info.value.point_count == point_count
    assert exc_info.value.minimum_point_count == 10


def test_food_point_threshold_must_be_positive():
    with pytest.raises(ValueError, match='at least 1'):
        food_presence.require_minimum_food_points([], 0)
