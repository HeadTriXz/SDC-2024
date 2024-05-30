import airsim
import numpy as np

from src.utils.lidar.ILidar import ILidar


class SimLidar(ILidar):
    """Class to read data from the lidar and process the data from it."""

    def __init__(self, client: airsim.CarClient) -> None:
        """Initializes the lidar."""
        super().__init__()
        self.client = client
        self.points = np.full(360, np.inf, dtype=np.float32)

    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        while self.running:
            lidar_data = self.client.getLidarData()
            if len(lidar_data.point_cloud) == 0:
                continue

            points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape((-1, 3))
            angles, distances = np.hsplit(
                np.rad2deg(
                    np.column_stack((np.arctan2(points[:, 1], points[:, 0]), np.linalg.norm(points[:, :2], axis=1)))
                ),
                2,
            )
            angles = angles.astype(int)
            distances = distances / 2
            distances[distances > 600] = np.inf

            # get the index of first occurence of each angle
            angle_starts = np.unique(angles, return_index=True)[1]
            updating_angles = (angles[angle_starts] + 180) % 360

            for i, angle in enumerate(updating_angles):
                self.points[angle] = distances[angle_starts[i]]


    def find_obstacle_distance(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the distance to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        if angle_min < 0:
            return min(*self.points[359 + angle_min :], *self.points[:angle_max])

        return min(self.points[angle_min:angle_max])

    def find_nearest_angle(self, angle_min: int, angle_max: int) -> int:
        """A function that finds the angle to the closest obstacle in a certain angle range.

        :param angle_min: The minimum angle to check.
        :param angle_max: The maximum angle to check.
        :return: The distance to the closest obstacle.
        """
        return np.argmin(self.points[angle_min:angle_max]) + angle_min



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

    def find_highest_index(self, angle_min: int, angle_max: int, min_dist: int, max_dist:int) -> int:
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

    def find_lowest_index(self, angle_min: int, angle_max: int, min_dist: int, max_dist:int) -> int:
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
