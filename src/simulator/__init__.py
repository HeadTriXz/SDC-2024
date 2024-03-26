from typing import Generator

import airsim
import cv2

import config
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.lane_assist import LaneAssist, PathFollower
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
            # remove the bottom 20 pixels
            if img_rotated is None:
                continue
            img_rotated = img_rotated[:-20, :]

            yield cv2.cvtColor(img_rotated, cv2.COLOR_BGR2GRAY)

    can_controller = SimCanController(client)
    speed_controller = SpeedController(can_controller)
    speed_controller.max_speed = 50
    speed_controller.state = SpeedControllerState.WAITING_TO_STOP

    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    lx = LaneAssist(
        get_sim_image_generator(),
        path_follower,
        speed_controller,
        adjust_speed=lambda _: config.requested_speed,
    )

    lx.start(True)
    input("Press enter to stop")
