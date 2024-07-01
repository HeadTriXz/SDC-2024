import airsim
import argparse
import cv2
import numpy as np

from typing import Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.constants import Gear
from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.lane_assist.stop_line_assist import StopLineAssist
from src.simulation.can_controller import SimCANController
from src.simulation.sim_lidar import SimLidar
from src.telemetry.app import TelemetryServer
from src.utils.parking import ParkingManoeuvre


def simulate_lane_assist(park: bool = False) -> None:
    """Simulate the lane assist.

    :param park: Whether to simulate the parking manoeuvre.
    """
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

    def get_sim_image_generator() -> Generator[np.ndarray, None, None]:
        while True:
            if park:
                yield np.zeros((height, width), dtype=np.uint8)
                continue

            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            response = responses[0]

            flat_image = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            rgb_image = flat_image.reshape(response.height, response.width, 3)
            rot_image = cv2.rotate(rgb_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if rot_image is None:
                continue

            grayscale = cv2.cvtColor(rot_image[:-20, :], cv2.COLOR_BGR2GRAY)
            thresholded = cv2.threshold(
                grayscale, config["preprocessing"]["white_threshold"], 255, cv2.THRESH_BINARY
            )[1]

            # Crop the image to the same size as real-world images.
            cx, cy = thresholded.shape[1] // 2, thresholded.shape[0]

            cropped = thresholded[cy - my:cy + my, cx - mx:cx + mx]
            resized = cv2.resize(cropped, (width, height))

            # Add information loss; this is the same as with real cameras.
            front_view = cv2.warpPerspective(
                resized,
                calibration.topdown_matrix,
                calibration.stitched_shape,
                flags=cv2.WARP_INVERSE_MAP | cv2.INTER_NEAREST
            )

            top_view = cv2.warpPerspective(
                front_view,
                calibration.topdown_matrix,
                calibration.output_shape,
                flags=cv2.INTER_NEAREST
            )

            if config["telemetry"]["enabled"] and telemetry.any_listening():
                telemetry.websocket_handler.send_image("topdown", top_view)

            yield top_view

    can_controller = SimCANController()
    speed_controller = SpeedController(can_controller)
    speed_controller.gear = Gear.DRIVE
    speed_controller.state = SpeedControllerState.DRIVING
    speed_controller.max_speed = 5

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
    lane_assist.start(multithreading=park)

    if park:
        lidar = SimLidar(client)
        lidar.start()

        manoeuvre = ParkingManoeuvre(lidar, lane_assist)
        manoeuvre.park()


def start_simulator() -> None:
    """Run the simulator."""
    parser = argparse.ArgumentParser(description="Run the simulator.")
    parser.add_argument("--park", action="store_true", help="Whether to simulate the parking manoeuvre.")

    args = parser.parse_args()

    simulate_lane_assist(args.park)
