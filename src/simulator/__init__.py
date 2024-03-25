from typing import Generator

import airsim
import cv2

from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist import LaneLynx, PathFollower
from simulator.simulator_can_controller import SimCanController


def main() -> None:
    """Run the simulator."""
    import numpy as np

    # startup the client
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
            yield cv2.cvtColor(img_rotated, cv2.COLOR_BGR2GRAY)

    can_controller = SimCanController(client)
    speed_controller = SpeedController(can_controller)
    speed_controller.max_speed = 10
    speed_controller.state = SpeedControllerState.DRIVING

    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 50.0

    lx = LaneLynx(
        get_sim_image_generator(),
        path_follower,
        speed_controller,
        adjust_speed=lambda _: 50,
    )

    lx.start()