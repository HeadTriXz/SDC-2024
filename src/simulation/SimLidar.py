import airsim
import numpy as np

from src.utils.lidar.ILidar import ILidar


class SimLidar(ILidar):
    """Class to read data from the lidar and process the data from it."""

    def __init__(self, client: airsim.CarClient) -> None:
        """Initializes the lidar."""
        super().__init__()
        self.client = client
        self._points = np.full(360, np.inf)

    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        while True:
            if not self.running:
                return None

            lidar_data = self.client.getLidarData()
            if len(lidar_data.point_cloud) == 0:
                return np.array([])
            points = np.array(lidar_data.point_cloud, dtype=np.float32)
            points = points.reshape((int(points.shape[0] / 3), 3))

            x_y = points[:, :2]
            distances = np.linalg.norm(x_y, axis=1)
            angles = np.arctan2(x_y[:, 1], x_y[:, 0])
            points = np.stack((angles, distances), axis=1)

            points = np.rad2deg(points)
            points = np.round(points)
            points = np.unique(points, axis=0)

            distances = np.full((360, 1), np.inf)
            for point in points:
                angle, distance = point
                angle = int(angle)
                if distances[angle] > distance:
                    distances[angle] = distance

            self._points = distances
