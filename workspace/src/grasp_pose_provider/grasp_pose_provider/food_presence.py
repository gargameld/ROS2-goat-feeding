"""Policy for deciding whether segmentation found enough food to grasp."""


# Do not ask GPD to find a grasp when segmentation produced too little evidence
# of food. This also filters isolated depth-camera noise.
DEFAULT_MIN_FOOD_POINT_COUNT = 10


class NoFoodDetectedError(RuntimeError):
    """Raised when segmentation finds too few food points for grasping."""

    def __init__(self, point_count, minimum_point_count):
        self.point_count = point_count
        self.minimum_point_count = minimum_point_count
        super().__init__(
            f'found {point_count} food points; at least '
            f'{minimum_point_count} are required'
        )


def require_minimum_food_points(food_indices, minimum_point_count):
    """Raise :class:`NoFoodDetectedError` for an empty/tiny segmentation."""
    if minimum_point_count < 1:
        raise ValueError('min_food_point_count must be at least 1.')

    point_count = len(food_indices)
    if point_count < minimum_point_count:
        raise NoFoodDetectedError(point_count, minimum_point_count)
