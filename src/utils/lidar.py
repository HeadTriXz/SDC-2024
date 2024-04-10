from math import floor
from adafruit_rplidar import RPLidar
from threading import Thread

import numpy as np


class Lidar:
    """A class to read data from the lidar and process the data from it."""

    def __init__(self, port_name: str) -> None:
        """Initializes the lidar.

        :param port_name: the name of the port
        """
        self.port_name = port_name
        self.lidar = RPLidar(None, self.port_name, timeout=3)
        self.max_distance = 0
        self.scan_data = [0]*360
        self.stopped = True
        self.thread = Thread(target=self.capture, daemon=True)

    def find_obstacle_distance(self, data: list[int], anglemin: int, anglemax: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param anglemin: the minimum angle to check
        :param anglemax: the maximum angle to check

        :return: the distance to the closest obstacle
        """
        point = 15000
        for i in range(anglemin, anglemax):
            if point > data[i] > 500:
                point = data[i]
        return point

    def right_free(self) -> bool:
        """A function that checks if the right side of the car is free.

        :return: True if the right side is free, False otherwise
        """
        if min(self.scan_data[20:120]) < 2500:
            for i in range(20, 120):
                if self.scan_data[i] < 2500 and(self.scan_data[i+1] < 2500 or self.scan_data[i-1] < 2500):
                    return False
        return True

    def free_side(self, anglemin: int, anglemax: int) -> bool:
        """A function that checks if the side between anglemin and anglemax of the car is free.

        :param side: the side to check, either "right" or "left"
        :return: True if the side is free, False otherwise
        """
        return all(
            not (
                    self.scan_data[i] < 2500 and (self.scan_data[i + 1] < 2500 or self.scan_data[i - 1] < 2500)
            ) for i in range(anglemin, anglemax)
        )

    def data_processing(self) -> list[int]:
        """A function that processes the data from the lidar.

        :return: the processed data
        """
        distances = np.array(self.scan_data)
        newdistances = distances.copy()
        for i in range(len(distances) - 1):
            if distances[i] < 500:
                newdistances[i] = 15000

            if np.abs(distances[i] - distances[i + 1]) > 1500:
                newdistances[i] = np.nan
        return newdistances

    def capture(self) -> None:
        """A function that captures the data from the lidar."""
        while not self.stopped:
            for scan in self.lidar.iter_scans():
                self.scan_data = [0]*360
                for (_, angle, distance) in scan:
                    self.scan_data[min([359, floor(angle)])] = distance

    def start(self) -> None:
        """Start the lidar."""
        self.stopped = False
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.stopped = True
        self.lidar.stop()
        self.lidar.disconnect()
