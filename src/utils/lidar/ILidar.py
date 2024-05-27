from threading import Thread

import numpy as np

from abc import ABC, abstractmethod


class ILidar(ABC):

    def __init__(self) -> None:
        self.thread = Thread(target=self.capture, daemon=True)
        self.__points = np.full(360, np.inf)

    @property
    def points(self) -> np.ndarray:
        """Return the points in the lidar sensor."""
        return self.__points

    @abstractmethod
    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        pass


    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.points[359 + angle_min :], *self.points[:angle_max])

        return min(self.points[angle_min:angle_max])

    def free_range(self, angle_min: int, angle_max: int, distance: int) -> bool:
        """A function that checks if the side between angle_min and angle_max of the car is free.

        :param angle_min: The minimum angle to check. (180 is the front of the car)
        :param angle_max: The maximum angle to check. (180 is the front of the car)
        :param distance: The minimum distance to check.
        :return: Whether the side is free.
        """
        return self.find_obstacle_distance(angle_min, angle_max) > distance

    def start(self) -> None:
        """Start the lidar."""
        self.running = True
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.running = False
