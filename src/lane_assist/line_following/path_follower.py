import numpy as np

from simple_pid import PID


class PathFollower:
    """A class to follow a path using a PID controller.

    Attributes
    ----------
        errors: A list of errors.
        look_ahead_distance: The distance to look ahead.
        max_steering_range: The max steering range.
        pid: The PID controller.

    """

    errors: list[float]
    look_ahead_distance: int
    max_steering_range: float
    pid: PID

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

        :param kp: The proportional gain.
        :param ki: The integral gain.
        :param kd: The derivative gain.
        :param setpoint: The setpoint of the PID controller.
        :param look_ahead_distance: The distance to look ahead.
        """
        self.pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=setpoint)
        self.look_ahead_distance = look_ahead_distance
        self.max_steering_range = max_steering_range
        self.errors = []

    def get_steering_fraction(self, path: np.ndarray, car_position: float) -> float:
        """Get the steering percentage to follow the path.

        :param path: The path to follow.
        :param car_position: The current x position.
        :return: The steering angle remapped to the range of -1 to 1.
        """
        angle = self.get_steering_angle(path, car_position)
        angle = np.clip(angle, -self.max_steering_range, self.max_steering_range)

        return angle / self.max_steering_range

    def get_steering_angle(self, path: np.ndarray, car_position: float) -> float:
        """Get the steering angle to follow the path.

        This function will use pid to get the steering angle to follow the path.
        It will not limit the angle to the max steering angle, it is the raw
        result from the PID controller.

        :param path: The path to follow.
        :param car_position: The current x position.
        :return: The steering angle to follow the path.
        """
        if path.shape[0] == 0:
            return 0

        target_point = path[-1] if path.shape[0] < self.look_ahead_distance else path[self.look_ahead_distance]

        # Get the distance to the target point
        x_distance_to_target = target_point[0] - car_position
        self.errors.append(x_distance_to_target)

        return -self.pid(x_distance_to_target)

    def update_pid(self, kp: float = None, ki: float = None, kd: float = None, setpoint: float = None) -> None:
        """Update the PID controller.

        :param kp: The proportional gain.
        :param ki: The integral gain.
        :param kd: The derivative gain.
        :param setpoint: The setpoint of the PID controller.
        """
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd
        if setpoint is not None:
            self.pid.setpoint = setpoint
