import airsim
import numpy as np

from threading import Thread

from src.utils.lidar import BaseLidar


class SimLidar(BaseLidar):
    """Class to read data from the lidar and process the data from it.

    Attributes
    ----------
        client: The airsim client.
        scan_data: The data from the lidar.
        thread: The thread that captures the data from the lidar.

    """

    client: airsim.CarClient
    scan_data: np.ndarray
    thread: Thread

    def __init__(self, client: airsim.CarClient) -> None:
        """Initializes the lidar."""
        super().__init__()
        self.client = client
        self.scan_data = np.full(360, np.inf, dtype=np.float32)
        self.thread = Thread(target=self.__listen, daemon=True)

    def start(self) -> None:
        """Start the lidar."""
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.thread.join()

    def __listen(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        while True:
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

            distances *= 13.5
            distances[distances > 6000] = np.inf

            angle_starts = np.unique(angles, return_index=True)[1]
            updating_angles = (angles[angle_starts] + 180) % 360

            for i, angle in enumerate(updating_angles):
                self.scan_data[angle] = distances[angle_starts[i]]
