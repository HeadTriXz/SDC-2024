import airsim
import cv2
import numpy as np

from pathlib import Path
from typing import Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist, PathFollower
from src.lane_assist.stopline_assist import StopLineAssist
from src.simulation.can_controller import SimCanController
from src.telemetry.app import TelemetryServer


def start_simulator() -> None:
    """Run the simulator."""
    telemetry = TelemetryServer()

    # Start the client.
    client = airsim.CarClient()
    client.confirmConnection()

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

    can_controller = SimCanController()
    speed_controller = SpeedController(can_controller)
    speed_controller.max_speed = 50
    speed_controller.state = SpeedControllerState.DRIVING

    # Load the calibration data
    calibration_file = Path(config.calibration.calibration_file)
    if not calibration_file.exists():
        raise FileNotFoundError(f"Calibration file not found: {calibration_file}")

    calibration = CalibrationData.load(calibration_file)

    # Initialize the path follower
    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    # Initialize the lane assist
    stop_line_assist = StopLineAssist(speed_controller, calibration)
    lane_assist = LaneAssist(
        get_sim_image_generator,
        stop_line_assist,
        path_follower,
        speed_controller,
        telemetry=telemetry,
        calibration=calibration
    )

    telemetry.start()
    speed_controller.start()
    lane_assist.start()
