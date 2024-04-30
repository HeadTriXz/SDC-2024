import os

from config import config
from constants import Gear, CameraResolution
from driving.can import CANController, get_can_bus
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.helpers import td_stitched_image_generator
from lane_assist.lane_assist import LaneAssist
from lane_assist.line_following.path_follower import PathFollower
from lane_assist.stopline_assist import StopLineAssist
from object_recognition.handlers.overtake_handler import OvertakeHandler
from object_recognition.handlers.pedestrian_handler import PedestrianHandler
from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from object_recognition.object_controller import ObjectController
from object_recognition.object_detector import ObjectDetector
from pathlib import Path
from utils.calibration_data import CalibrationData
from utils.lidar import Lidar
from utils.video_stream import VideoStream
from telemetry.app import TelemetryServer


def start_kart() -> None:
    """Start the main loop."""
    cam_left = VideoStream(config.camera_ids.left, resolution=CameraResolution.NHD)
    cam_center = VideoStream(config.camera_ids.center, resolution=CameraResolution.HD)
    cam_right = VideoStream(config.camera_ids.right, resolution=CameraResolution.NHD)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    # FIXME: remove telemetry
    telemetry_server = TelemetryServer()

    # Connect to CAN bus
    bus = get_can_bus()
    can_controller = CANController(bus)
    speed_controller = SpeedController(can_controller)

    # Initialize the speed controller
    speed_controller.gear = Gear.DRIVE
    speed_controller.state = SpeedControllerState.DRIVING
    speed_controller.max_speed = 50

    # Initialize the path follower
    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    # Load the calibration data
    calibration_file = Path(config.calibration.calibration_file)
    if not calibration_file.exists():
        raise FileNotFoundError(f"Calibration file not found: {calibration_file}")

    calibration = CalibrationData.load(calibration_file)

    # Initialize the lane assist
    stop_line_assist = StopLineAssist(speed_controller, calibration)
    lane_assist = LaneAssist(
        td_stitched_image_generator(calibration, cam_left, cam_center, cam_right, telemetry_server),
        stop_line_assist,
        path_follower,
        speed_controller,
        telemetry=telemetry_server,
        calibration=calibration
    )

    # Initialize the object detector
    controller = ObjectController(calibration, lane_assist, speed_controller)
    controller.add_handler(PedestrianHandler(controller))
    controller.add_handler(SpeedLimitHandler(controller))
    controller.add_handler(TrafficLightHandler(controller))

    lidar = Lidar.safe_init()
    if lidar is not None:
        controller.add_handler(OvertakeHandler(controller, lidar))

    detector = ObjectDetector.from_model(config.object_detection.model_path, controller, config.camera_ids.center)

    # Start the system
    can_controller.start()
    speed_controller.start()
    detector.start()
    telemetry_server.start()
    lane_assist.start()


if __name__ == "__main__":
    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from simulation.main import start_simulator

        start_simulator()
    else:
        start_kart()
