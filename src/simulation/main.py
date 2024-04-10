import os

import airsim
import cv2
import numpy as np

from config import config
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.lane_assist import LaneAssist, PathFollower
from simulation.can_controller import SimCanController
from telemetry.app import TelemetryServer
from typing import Generator


def start_simulator() -> None:
    """Run the simulator."""
    telemetry = TelemetryServer()

    # Start the client.
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.reset()

    def get_sim_image_generator() -> Generator[np.ndarray, None, None]:
        while True:
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            response = responses[0]

            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            img_rotated = cv2.rotate(img_rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if img_rotated is None:
                continue

            grayscale = cv2.cvtColor(img_rotated[:-20, :], cv2.COLOR_BGR2GRAY)
            if config.telemetry.enabled:
                telemetry.websocket_handler.send_image("topdown", grayscale)

            yield grayscale

    can_controller = SimCanController(client)
    speed_controller = SpeedController(can_controller)
    speed_controller.max_speed = 50
    speed_controller.state = SpeedControllerState.DRIVING

    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    lane_assist = LaneAssist(
        get_sim_image_generator,
        path_follower,
        speed_controller,
        adjust_speed=lambda _path: 15,
        telemetry=telemetry,
    )

    lane_assist.start(True)
    telemetry.start()

    input("Press enter to stop")

    # write the pickled errors and fps into a file
    import pickle
    import time

    folder_path = "../data/telemetry/"
    os.makedirs(folder_path)

    with open(f"{folder_path}{time.time()}-errors.pkl", "wb") as f:
        pickle.dump(lane_assist.path_follower.errors, f)

    # write the fps into a file
    with open(f"{folder_path}{time.time()}-frame_times.pkl", "wb") as f:
        pickle.dump(lane_assist.frame_times, f)
