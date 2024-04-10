import numpy as np


def get_coefficient_of_friction(speed: float, radius: float) -> float:
    """Get the coefficient of friction."""
    return speed**2 / (9.81 * radius)


def get_max_corner_speed(radius: float, friction_coefficient: float = 0.9) -> float:
    """Get the max speed around a circle."""
    return np.sqrt(friction_coefficient * 9.81 * radius)


def get_radius_of_path(_path: np.ndarray) -> set[float, float, float]:
    """Get the radius of curvature."""
    raise NotImplementedError('The function "get_radius_of_path" is not implemented yet.')
