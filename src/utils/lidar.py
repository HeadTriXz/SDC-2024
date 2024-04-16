import numpy as np

from config import config
from rplidar import RPLidar
from math import floor
from threading import Thread


class Lidar:
    """Class to read data from the lidar and process the data from it.

    The lidar can be used to find the distance to the obstacles around the car.
    """

    scan_data = np.full(360, np.inf)
    running = False

    def __init__(self) -> None:
        """Initializes the lidar.

        :param port_name: The name of the port.
        """
        self.lidar = RPLidar(config.lidar.port_name, timeout=3)
        self.thread = Thread(target=self.capture, daemon=True)

    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.scan_data[359 + angle_min:], *self.scan_data[:angle_max])

        return min(self.scan_data[angle_min:angle_max])

    def free_range(self, angle_min: int, angle_max: int, distance: int) -> bool:
        """A function that checks if the side between angle_min and angle_max of the car is free.

        :param angle_min: The minimum angle to check. (180 is the front of the car)
        :param angle_max: The maximum angle to check. (180 is the front of the car)
        :param distance: The minimum distance to consider the side free.
        :return: True if the side is free, False otherwise.
        """
        return self.find_obstacle_distance(angle_min, angle_max) > distance

    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        for scan in self.lidar.iter_scans():
            if not self.running:
                return

            for i, (_, angle, distance) in enumerate(scan, 1):
                if distance < config.lidar.min_distance:
                    self.scan_data[floor(angle)] = np.inf
                    continue

                prev_diff = abs(self.scan_data[i] - self.scan_data[i - 1])
                next_diff = abs(self.scan_data[i] - self.scan_data[i + 1])

                prev_larger = prev_diff > config.lidar.max_distance_between_points
                next_larger = next_diff > config.lidar.max_distance_between_points

                if (i > 0 and prev_larger) or ((len(scan) - i) > 0 and next_larger):
                    self.scan_data[floor(angle)] = np.inf
                    continue

                self.scan_data[floor(angle)] = distance

    def start(self) -> None:
        """Start the lidar."""
        self.running = True
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.running = False

