import numpy as np


def get_coefficient_of_friction(speed: float, radius: float) -> float:
    """Get the coefficient of friction."""
    return speed**2 / (9.81 * radius)


def get_max_corner_speed(radius: float, friction_coefficient: float = 0.9) -> float:
    """Get the max speed around a circle."""
    # the max speed is calculated using the friction coefficient
    return np.sqrt(friction_coefficient * 9.81 * radius)
