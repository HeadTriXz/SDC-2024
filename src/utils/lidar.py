import logging
import numpy as np

from math import floor
from rplidar import RPLidar
from threading import Thread
from typing import Generator, Optional

from src.config import config


class Lidar:
    """Class to read data from the lidar and process the data from it.

    The lidar can be used to find the distance to the obstacles around the car.

    Attributes
    ----------
        lidar: The lidar object.
        scan_data: The data from the lidar.
        thread: The thread that captures the data from the lidar.

    """

    lidar: RPLidar
    scan_data: np.ndarray
    thread: Thread

    def __init__(self) -> None:
        """Initializes the lidar."""
        self.lidar = RPLidar(config["lidar"]["port_name"], timeout=3)
        self.lidar.stop_motor()

        self.thread = Thread(target=self.__listen, daemon=True)
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

    def free_range(self, angle_min: int, angle_max: int, distance: int, max_points: int = 0) -> bool:
        """A function that checks if the side between angle_min and angle_max of the car is free.

        :param angle_min: The minimum angle to check. (180 is the front of the car)
        :param angle_max: The maximum angle to check. (180 is the front of the car)
        :param distance: The minimum distance to check.
        :param max_points: The maximum allowed points in the range (to avoid false positives).
        :return: Whether the side is free.
        """
        distances = self.scan_data[angle_min:angle_max]
        within_range = np.sum(distances < distance)

        return within_range <= max_points

    def start(self) -> None:
        """Start the lidar."""
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.thread.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def __capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        self.lidar.start_motor()
        for scan in self.__iter_scans():
            for _, angle, distance in scan:
                angle = min(359, floor(angle))
                if distance < config["lidar"]["min_distance"]:
                    self.scan_data[angle] = np.inf
                    continue

                self.scan_data[angle] = distance

    def __iter_scans(self) -> Generator[np.ndarray, None, None]:
        """Yield the scans from the lidar.

        :return: The scans from the lidar.
        """
        scan_list = []
        for new_scan, quality, angle, distance in self.lidar.iter_measures():
            if new_scan:
                if len(scan_list) > 5:
                    yield scan_list
                scan_list = []

            scan_list.append((quality, angle, distance))

    def __listen(self) -> None:
        """Listen for data from the lidar sensor."""
        while True:
            try:
                self.__capture()
            except Exception as e:
                logging.error("Failed to capture data from the lidar: %s", e)

                self.lidar.stop()
                self.lidar.stop_motor()

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
