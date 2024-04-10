from config import config
from constants import Gear, CameraResolution
from driving.can import CANController, get_can_bus
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.helpers import td_stitched_image_generator
from lane_assist.lane_assist import LaneAssist
from lane_assist.line_following.path_follower import PathFollower
from lane_assist.preprocessing.calibrate import CameraCalibrator
from object_recognition.handlers.pedestrian_handler import PedestrianHandler
from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from object_recognition.object_controller import ObjectController
from object_recognition.object_detector import ObjectDetector
from pathlib import Path
from utils.video_stream import VideoStream
from telemetry.app import TelemetryServer


def calibrate_cameras() -> None:
    """Example script to calibrate the cameras."""
    cam_left = VideoStream(config.camera_ids.left, resolution=CameraResolution.FHD)
    cam_center = VideoStream(config.camera_ids.center, resolution=CameraResolution.FHD)
    cam_right = VideoStream(config.camera_ids.right, resolution=CameraResolution.FHD)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    if not cam_left.has_next() or not cam_center.has_next() or not cam_right.has_next():
        raise ValueError("Could not capture images from cameras")

    left_image = cam_left.next()
    center_image = cam_center.next()
    right_image = cam_right.next()

    calibrator = CameraCalibrator([left_image, center_image, right_image], input_shape=(1280, 720))
    calibrator.calibrate()
    calibrator.save(config.calibration.save_dir)

    cam_left.stop()
    cam_center.stop()
    cam_right.stop()


def start_kart() -> None:
    """Start the main loop."""
    cam_left = VideoStream(config.camera_ids.left)
    cam_center = VideoStream(config.camera_ids.center)
    cam_right = VideoStream(config.camera_ids.right)

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
    speed_controller.state = SpeedControllerState.WAITING_TO_STOP
    speed_controller.max_speed = 50

    # Initialize the path follower
    path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    # # Initialize the object controller
    controller = ObjectController(speed_controller)
    controller.add_handler(PedestrianHandler(controller))
    controller.add_handler(SpeedLimitHandler(controller))
    controller.add_handler(TrafficLightHandler(controller))

    # Load the calibration data
    calibration_file = Path(config.calibration.calibration_file)
    if not calibration_file.exists():
        raise FileNotFoundError(f"Calibration file not found: {config.calibration.calibration_file}")

    calibrator = CameraCalibrator.load(calibration_file)

    # Initialize the lane assist
    lane_assist = LaneAssist(
        td_stitched_image_generator(calibrator, cam_left, cam_center, cam_right, telemetry_server),
        path_follower,
        speed_controller,
        adjust_speed=lambda _: 1,
        telemetry=telemetry_server,
    )

    # Initialize the object detector
    detector = ObjectDetector.from_model(config.object_detection.model_path, controller, config.camera_ids.center)

    # Start the system
    can_controller.start()
    speed_controller.start()
    detector.start()
    telemetry_server.start()
    lane_assist.start()

    input("Press Enter to stop...")


if __name__ == "__main__":
    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulation":
        from simulation.main import start_simulator
        start_simulator()
    else:
        start_kart()
