import numpy as np

from abc import ABC, abstractmethod


class BaseLidar(ABC):
    """Interface for the lidar classes.

    Attributes
    ----------
        scan_data: A numpy array containing the scan data from the lidar.

    """

    scan_data: np.ndarray

    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.scan_data[359 + angle_min :], *self.scan_data[:angle_max])

        return min(self.scan_data[angle_min:angle_max])

    def find_rightmost_point(self, angle_min: int, angle_max: int, min_dist: int, max_dist: int) -> float:
        """A function that returns the distance to rightmost object in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param min_dist: The distance threshold to check.
        :param max_dist: The distance threshold to check.
        :return: The distance to the closest obstacle.
        """
        for i in range(angle_max, angle_min, -1):
            if min_dist < self.scan_data[i] < max_dist:
                return self.scan_data[i]

        return np.inf

    def find_nearest_angle(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the angle to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if np.all(np.isinf(self.scan_data[angle_min:angle_max])):
            return -1

        return np.argmin(self.scan_data[angle_min:angle_max]) + angle_min

    def find_highest_index(self, angle_min: int, angle_max: int, min_dist: int, max_dist: int) -> int:
        """A function that returns the highest index with a distance in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param min_dist: The minimum distance to check.
        :param max_dist: The maximum distance to check.
        :return: The highest index with a distance in range.
        """
        for i in range(angle_max, angle_min, -1):
            if min_dist < self.scan_data[i] < max_dist:
                return i

        return -1

    def find_lowest_index(self, angle_min: int, angle_max: int, min_dist: int, max_dist: int) -> int:
        """A function that returns the lowest index with a distance in range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :param min_dist: The minimum distance to check.
        :param max_dist: The maximum distance to check.
        :return: The lowest index with a distance in range.
        """
        for i in range(angle_min, angle_max):
            if min_dist < self.scan_data[i] < max_dist:
                return i

        return -1

    def free_range(self, angle_min: int, angle_max: int, distance: int) -> bool:
        """A function that checks if the side between angle_min and angle_max of the car is free.

        :param angle_min: The minimum angle to check. (180 is the front of the car)
        :param angle_max: The maximum angle to check. (180 is the front of the car)
        :param distance: The minimum distance to check.
        :return: Whether the side is free.
        """
        return self.find_obstacle_distance(angle_min, angle_max) > distance

    @abstractmethod
    def start(self) -> None:
        """Start the lidar."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop the lidar."""
        pass
