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
