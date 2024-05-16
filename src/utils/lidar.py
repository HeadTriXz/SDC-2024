import logging
import numpy as np

from math import floor
from rplidar import RPLidar
from threading import Thread
from typing import Optional

from src.config import config


class Lidar:
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
        self.lidar = RPLidar(config.lidar.port_name, timeout=5)
        self.thread = Thread(target=self.capture, daemon=True)
        self.scan_data = np.full(360, np.inf)

    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.scan_data[359 + angle_min :], *self.scan_data[:angle_max])

        return min(self.scan_data[angle_min:angle_max])

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
                if distance < config.lidar.min_distance:
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
