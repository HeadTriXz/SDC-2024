from typing import Any

import numpy as np
from simple_pid import PID


class PathFollower:
    """A class to follow a path using a PID controller."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0,
        look_ahead_distance: int = 10,
        max_steering_range: float = 30.0,
    ) -> None:
        """Initialize the LineFollowing class.

        Parameters
        ----------
        :param kp: the proportional gain.
        :param ki: the integral gain.
        :param kd: the derivative gain.
        :param setpoint: the setpoint of the PID controller.
        :param look_ahead_distance: the distance to look ahead.

        """
        self.pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=setpoint)
        self.look_ahead_distance = look_ahead_distance
        self.max_steering_range = max_steering_range

    def get_steering_fraction(self, path: np.ndarray, car_position: float) -> float:
        """Get the steering percentage to follow the path.

        this is will return the steering angle remapped to the range of -1 to 1.

        Parameters
        ----------
        :param path: the path to follow.
        :param car_position: the current x position.

        Returns
        -------
        :return: the steering percentage to follow the path.

        """
        angle = self.get_steering_angle(path, car_position)
        angle = np.clip(angle, -self.max_steering_range, self.max_steering_range)
        return angle / self.max_steering_range

    def get_steering_angle(self, path: np.ndarray, car_position: float) -> float:
        """Get the steering angle to follow the path.

        This function will use pid to get the steering angle to follow the path.
        It will not limit the angle to the max steering angle, it is the raw
        result from the PID controller.

        Parameters
        ----------
        :param path: the path to follow.
        :param car_position: the current x position.

        Returns
        -------
        :return: the steering angle to follow the path.

        """
        if path.shape[0] == 0:
            return 0

        target_point = path[-1] if path.shape[0] < self.look_ahead_distance else path[self.look_ahead_distance]

        # get the y distacne to the target point
        x_distance_to_target: int | Any = target_point[0] - car_position
        return -self.pid(x_distance_to_target)
        # limit the angle to the max steering angle.

    def update_pid(
        self, kp: float | None = None, ki: float | None = None, kd: float | None = None, setpoint: float | None = None
    ) -> None:
        """Update the PID controller.

        Parameters
        ----------
        :param kp: the proportional gain.
        :param ki: the integral gain.
        :param kd: the derivative gain.

        """
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd
        if setpoint is not None:
            self.pid.setpoint = setpoint
