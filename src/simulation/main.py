import airsim
import cv2
import matplotlib.pyplot as plt
import numpy as np

from typing import Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.telemetry.app import TelemetryServer


def start_simulator() -> None:
    """Run the simulator."""
    telemetry = TelemetryServer()

    # Start the client.
    client = airsim.CarClient()
    client.confirmConnection()

    # Load the calibration data
    calibration = CalibrationData.load(config["calibration"]["calibration_file"])

    factor = 2.0
    height = calibration.output_shape[1]
    width = calibration.output_shape[0]

    mx, my = int(width * factor / 2), int(height * factor)

    def get_lidar_data() -> np.ndarray:
        lidar_data = client.getLidarData()
        # convert the point cloud to [(angle, distance), ...] in the car's frame of reference in horizontal axis
        if len(lidar_data.point_cloud) == 0:
            return np.array([])
        points = np.array(lidar_data.point_cloud, dtype=np.float32)
        points = points.reshape((int(points.shape[0] / 3), 3))

        x_y = points[:, :2]
        # get distance from the origin
        distances = np.linalg.norm(x_y, axis=1)
        # get the angle
        angles = np.arctan2(x_y[:, 1], x_y[:, 0])
        # combine the angles and distances
        points = np.stack((angles, distances), axis=1)

        # put into buckets of 1 degeree and grab a random point from each bucket
        # this is to reduce the number of points we have to process
        points = np.rad2deg(points)
        points = np.round(points)
        points = np.unique(points, axis=0)

        distances = np.full((360, 1), np.inf)
        for point in points:
            angle, distance = point
            angle = int(angle)
            if distances[angle] > distance:
                distances[angle] = distance

        return distances

    def get_sim_image_generator() -> Generator[np.ndarray, None, None]:
        while True:
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            response = responses[0]

            flat_image = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            rgb_image = flat_image.reshape(response.height, response.width, 3)
            rot_image = cv2.rotate(rgb_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if rot_image is None:
                continue

            grayscale = cv2.cvtColor(rot_image[:-20, :], cv2.COLOR_BGR2GRAY)
            thresholded = cv2.threshold(grayscale, config["preprocessing"]["white_threshold"], 255, cv2.THRESH_BINARY)[
                1
            ]

            # Crop the image to the same size as real-world images.
            cx, cy = thresholded.shape[1] // 2, thresholded.shape[0]

            cropped = thresholded[cy - my : cy + my, cx - mx : cx + mx]
            resized = cv2.resize(cropped, (width, height))

            # Add information loss; this is the same as with real cameras.
            front_view = cv2.warpPerspective(
                resized,
                calibration.topdown_matrix,
                calibration.stitched_shape,
                flags=cv2.WARP_INVERSE_MAP | cv2.INTER_NEAREST,
            )

            top_view = cv2.warpPerspective(
                front_view, calibration.topdown_matrix, calibration.output_shape, flags=cv2.INTER_NEAREST
            )

            if config["telemetry"]["enabled"]:
                telemetry.websocket_handler.send_image("topdown", top_view)

            yield top_view

    while True:
        print("Getting lidar data")
        # plot the lidar data
        lidar_data = get_lidar_data()
        print(len(lidar_data))
        if len(lidar_data) == 0:
            continue

        # update the plot
        plt.clf()
        plt.polar(np.linspace(0, 2 * np.pi, 360), lidar_data)
        plt.pause(0.01)

