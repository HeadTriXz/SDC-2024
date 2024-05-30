import logging
import numpy as np

from math import floor
from rplidar import RPLidar
from threading import Thread
from typing import Optional

from src.config import config
from src.utils.lidar.ILidar import ILidar


class Lidar(ILidar):
    """Class to read data from the lidar and process the data from it.

    The lidar can be used to find the distance to the obstacles around the car.

    Attributes
    ----------
        lidar: The lidar object.
        running: Whether the lidar is running.
        scan_data: The data from the lidar.
        thread: The thread that captures the data from the lidar.

    """

    lidar: RPLidar
    running: bool = False
    scan_data: np.ndarray
    thread: Thread

    def __init__(self) -> None:
        """Initializes the lidar."""
        super().__init__()

        self.lidar = RPLidar(config["lidar"]["port_name"], timeout=5)
        self.scan_data = np.full(360, np.inf)

    @property
    def points(self) -> np.ndarray:
        """Return the points in the lidar sensor."""
        return self.scan_data

    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.scan_data[359 + angle_min :], *self.scan_data[:angle_max])

        return min(self.scan_data[angle_min:angle_max])

    def find_rightmost_point(self, angle_min: int, angle_max: int, min_dist: int, max_dist: int) -> int:
        """A function that returns the distance to rightmost object in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param max_dist: The distance threshold to check.
        :return: The distance to the closest obstacle.
        """
        for i in range(angle_max, angle_min, -1):
            if min_dist < self.points[i] < max_dist:
                return self.points[i]
        return 0

    def find_nearest_angle(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the angle to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        return np.argmin(self.points[angle_min:angle_max]) + angle_min

    def find_highest_angle(self, angle_min: int, angle_max: int, min_dist: int, max_dist:int) -> int:
        """A function that returns the highest index with a distance in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param min_dist: The minimum distance to check.
        :param max_dist: The maximum distance to check.
        :return: The highest index with a distance in range.
        """
        for i in range(angle_max, angle_min, -1):
            if min_dist < self.points[i] < max_dist:
                return i
        return 0

    def find_lowest_angle(self, angle_min: int, angle_max: int, min_dist: int, max_dist:int) -> int:
        """A function that returns the lowest index with a distance in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param min_dist: The minimum distance to check.
        :param max_dist: The maximum distance to check.
        :return: The lowest index with a distance in range.
        """
        for i in range(angle_min, angle_max):
            if min_dist < self.points[i] < max_dist:
                return i
        return 0

    def free_range(self, angle_min: int, angle_max: int, distance: int) -> bool:
        """A function that checks if the side between angle_min and angle_max of the car is free.

        :param angle_min: The minimum angle to check. (180 is the front of the car)
        :param angle_max: The maximum angle to check. (180 is the front of the car)
        :param distance: The minimum distance to check.
        :return: Whether the side is free.
        """
        return self.find_obstacle_distance(angle_min, angle_max) > distance

    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        for scan in self.lidar.iter_scans():
            if not self.running:
                return

            for _, angle, distance in scan:
                angle = int(angle % 360)
                if distance < config["lidar"]["min_distance"]:
                    self.scan_data[floor(angle)] = np.inf
                    continue

                self.scan_data[floor(angle)] = distance

    def start(self) -> None:
        """Start the lidar."""
        self.running = True
        self.lidar.reset()
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.running = False

    @classmethod
    def safe_init(cls) -> Optional["Lidar"]:
        """Get the lidar sensor.

        :return: The lidar sensor.
        """
        try:
            return cls()
        except Exception as e:
            logging.warning("Failed to initialize the lidar: %s", e)
            return None
