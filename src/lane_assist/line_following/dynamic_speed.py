import numpy as np

from src.config import config
from src.lane_assist.line_following.path_generator import Path


def get_max_corner_speed(radius: float, friction_coefficient: float = 0.9) -> float:
    """Get the max speed around a circle in meters per second.

    :param radius: The radius of the circle.
    :param friction_coefficient: The friction coefficient of the road.
    :return: The max speed around the circle in m/s.
    """
    return np.sqrt(friction_coefficient * 9.81 * radius)


def get_max_path_speed(path: Path) -> int:
    """Get the max speed on the path in meters per second.

    :param path: The path to get the max speed from.
    :return: The max speed on the path in km/h.
    """
    if config.dynamic_speed.static:
        return config.dynamic_speed.static_speed

    return int(get_max_corner_speed(path.radius, config.dynamic_speed.friction_coefficient) * 3.6)
