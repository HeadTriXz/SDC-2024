import airsim
import cv2
import numpy as np

from typing import Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.constants import Gear
from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.lane_assist.stop_line_assist import StopLineAssist
from src.simulation.can_controller import SimCanController
from src.telemetry.app import TelemetryServer


def start_simulator() -> None:
    """Run the simulator."""
    telemetry = TelemetryServer()

    # Start the client.
    client = airsim.CarClient()
    client.confirmConnection()

    # Load the calibration data
    calibration = CalibrationData.load(config.calibration.calibration_file)

    factor = 2.0
    height = calibration.output_shape[1]
    width = calibration.output_shape[0]

    mx, my = int(width * factor / 2), int(height * factor)

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
            grayscale = cv2.threshold(grayscale, config.preprocessing.white_threshold, 255, cv2.THRESH_BINARY)[1]

            cx, cy = grayscale.shape[1] // 2, grayscale.shape[0]

            grayscale = grayscale[cy - my:cy + my, cx - mx:cx + mx]
            grayscale = cv2.resize(grayscale, (width, height))

            if config.telemetry.enabled:
                telemetry.websocket_handler.send_image("topdown", grayscale)

            yield grayscale

    can_controller = SimCanController()
    speed_controller = SpeedController(can_controller)
    speed_controller.gear = Gear.DRIVE
    speed_controller.state = SpeedControllerState.DRIVING
    speed_controller.max_speed = 50

    # Initialize the lane assist
    stop_line_assist = StopLineAssist(speed_controller, calibration)
    lane_assist = LaneAssist(
        get_sim_image_generator,
        stop_line_assist,
        speed_controller,
        telemetry=telemetry,
        calibration=calibration
    )

    telemetry.start()
    speed_controller.start()
    speed_controller.toggle()

    lane_assist.toggle()
    lane_assist.start()
