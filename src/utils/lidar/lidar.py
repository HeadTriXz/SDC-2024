import logging
import numpy as np

from math import floor
from rplidar import RPLidar
from threading import Thread
from typing import Generator, Optional

from src.config import config
from src.utils.lidar import BaseLidar


class Lidar(BaseLidar):
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
