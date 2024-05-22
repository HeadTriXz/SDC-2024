import numpy as np
import time

from simple_pid import PID

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.speed_controller import ISpeedController


class PathFollower:
    """A class to follow a path using a PID controller.

    Attributes
    ----------
        look_ahead_padding: The minimum distance to look ahead in meters
        max_steering_range: The max steering range.

    """

    look_ahead_padding: int
    max_steering_range: float
    __calibration: CalibrationData
    __speed_controller: ISpeedController
    __pid: PID

    def __init__(
        self,
        calibration: CalibrationData,
        speed_controller: ISpeedController,
    ) -> None:
        """Initialize the LineFollowing class.

        :param calibration: The calibration data.
        :param speed_controller: The speed controller.
        """
        self.max_steering_range = config["kart"]["max_steering_angle"]
        self.__calibration = calibration
        self.__speed_controller = speed_controller

        self.__pid = PID(
            Kp=config["line_following"]["pid"]["kp"],
            Ki=config["line_following"]["pid"]["ki"],
            Kd=config["line_following"]["pid"]["kd"],
            setpoint=0,
            output_limits=(-config["kart"]["max_steering_angle"], config["kart"]["max_steering_angle"]),
        )

    def __calc_lookahead_padding(self) -> float:
        """Get distance traveled since last PID update.

        :return: The distance traveled in pixels.
        """
        min_dist = self.__calibration.get_pixels(config["line_following"]["look_ahead_distance"])

        lt = self.__pid._last_time
        if lt is None:
            return min_dist

        # Get the distance we have traveled
        dt = time.monotonic() - lt
        speed = self.__speed_controller.current_speed / 3.6
        distance = self.__calibration.get_pixels(speed * dt)

        return min_dist + distance

    def get_path_point(self, path: np.ndarray) -> np.ndarray:
        """Get the point on the path to follow.

        :param path: The path to follow.
        :return: The point on the path to follow.
        """
        look_ahead = self.__calc_lookahead_padding()

        cum_distance = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        intersect_id = np.argmax(cum_distance >= look_ahead)

        return path[intersect_id]

    def get_steering_fraction(self, target_point: np.ndarray, car_position: float) -> float:
        """Get the steering percentage to follow the path.

        :param target_point: The target point on the path.
        :param car_position: The current x position.
        :return: The steering angle remapped to the range of -1.25 to 1.25.
        """
        angle = self.get_steering_angle(target_point, car_position)
        return angle / self.max_steering_range * 1.25

    def get_steering_angle(self, target_point: np.ndarray, car_position: float) -> float:
        """Get the steering angle to follow the path.

        This function will use pid to get the steering angle to follow the path.
        It will not limit the angle to the max steering angle, it is the raw
        result from the PID controller.

        :param target_point: The target point on the path.
        :param car_position: The current x position.
        :return: The steering angle to follow the path.
        """
        distance = self.__calibration.get_distance(target_point[0] - car_position)
        return -self.__pid(distance)

    def reset(self) -> None:
        """Reset the PID controller."""
        self.__pid.reset()
