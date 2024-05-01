import os

from config import config
from constants import Gear
from driving.can import CANController, get_can_bus
from driving.gamepad.driving_controller import BasicControllerDriving
from driving.gamepad.gamepad import EventType, Gamepad, GamepadButton
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
from utils.video_stream import VideoStream
from telemetry.app import TelemetryServer


class Kart:
    """Represents the system controlling a kart's operation.

    This class initializes and orchestrates various components involved in controlling a kart, including camera streams,
    telemetry server, CAN bus communication, speed controller, driving controller, lane assist, object detector,
    and handlers for different objects like pedestrians, speed limits, traffic lights, and overtaking scenarios.
    """

    def __init__(self) -> None:
        """Initialize the KartSystem.

        This method sets up the necessary components for controlling the kart.
        """
        # Initialize camera streams
        self.cam_left = VideoStream(config.camera_ids.left)
        self.cam_center = VideoStream(config.camera_ids.center)
        self.cam_right = VideoStream(config.camera_ids.right)

        # FIXME: remove telemetry
        self.telemetry_server = TelemetryServer()

        # Connect to CAN bus
        bus = get_can_bus()
        self.can_controller = CANController(bus)
        self.speed_controller = SpeedController(self.can_controller)

        # Initialize the speed controller
        self.speed_controller.gear = Gear.DRIVE
        self.speed_controller.state = SpeedControllerState.WAITING_TO_STOP
        self.speed_controller.max_speed = 50

        # Initialize the path follower
        self.path_follower = PathFollower(1, 0.01, 0.05, look_ahead_distance=10)
        self.path_follower.max_steering_range = 30.0

        # Load the calibration data
        calibration_file = Path(config.calibration.calibration_file)
        if not calibration_file.exists():
            raise FileNotFoundError(f"Calibration file not found: {calibration_file}")

        self.calibration = CalibrationData.load(calibration_file)

        # Initialize the gamepad
        self.gamepad = Gamepad()

        # Initialize the driving controller
        self.driving_controller = BasicControllerDriving(self.gamepad, self.can_controller)

        # Initialize the lane assist
        self.stop_line_assist = StopLineAssist(self.speed_controller, self.calibration)
        self.lane_assist = LaneAssist(
            td_stitched_image_generator(
                self.calibration, self.cam_left, self.cam_center, self.cam_right, self.telemetry_server
            ),
            self.stop_line_assist,
            self.path_follower,
            self.speed_controller,
            adjust_speed=lambda _: 1,
            telemetry=self.telemetry_server,
        )

        # Initialize the object detector
        self.controller = ObjectController(self.calibration, self.lane_assist, self.speed_controller)
        self.controller.add_handler(PedestrianHandler(self.controller))
        self.controller.add_handler(SpeedLimitHandler(self.controller))
        self.controller.add_handler(TrafficLightHandler(self.controller))
        self.controller.add_handler(OvertakeHandler(self.controller))

        self.detector = ObjectDetector.from_model(
            config.object_detection.model_path, self.controller, config.camera_ids.center
        )

    def switch_driving_mode(self, input_type=None, event_type=None, value=None) -> None:
        """Switch the driving mode of the kart."""
        self.gamepad.vibrate(500)
        current = self.driving_controller.paused
        self.driving_controller.paused = not current
        self.lane_assist.paused = current

    def start(self) -> None:
        """Start the kart system."""
        try:
            # Listener to switch driving mode
            self.gamepad.add_listener(GamepadButton.START, EventType.LONG_PRESS, self.switch_driving_mode)
            self.gamepad.start()

            # Start the system
            self.cam_left.start()
            self.cam_center.start()
            self.cam_right.start()
            self.can_controller.start()
            self.speed_controller.start()
            self.detector.start()
            self.telemetry_server.start()

            # Start the driving modes
            self.driving_controller.start()
            self.lane_assist.start()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from simulation.main import start_simulator

        start_simulator()
    else:
        kart = Kart()
        kart.start()
