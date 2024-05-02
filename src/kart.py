import time
from typing import Any

from src.config import config
from src.driving.can import get_can_bus, CANController
from src.driving.gamepad.driving_controller import BasicControllerDriving
from src.driving.gamepad.gamepad import Gamepad, GamepadButton, EventType
from src.driving.speed_controller import SpeedController
from src.lane_assist.helpers import td_stitched_image_generator
from src.lane_assist.lane_assist import LaneAssist
from src.lane_assist.line_following.path_follower import PathFollower
from src.lane_assist.stopline_assist import StopLineAssist
from src.object_recognition.handlers.overtake_handler import OvertakeHandler
from src.object_recognition.handlers.pedestrian_handler import PedestrianHandler
from src.object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from src.object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from src.object_recognition.object_controller import ObjectController
from src.object_recognition.object_detector import ObjectDetector
from src.telemetry.app import TelemetryServer
from src.utils.calibration_data import CalibrationData
from src.utils.lidar import Lidar
from src.utils.video_stream import VideoStream


class Kart:
    """A class to represent the kart.

    Attributes
    ----------
        can_controller (CANController): The CAN controller to use.
        speed_controller (SpeedController): The speed controller to use.
        cam_left (VideoStream): The left camera stream.
        cam_center (VideoStream): The center camera stream.
        cam_right (VideoStream): The right camera stream.
        lane_assist (LaneAssist): The lane assist to use.
        detector (ObjectDetector): The object detector to use.
        gamepad (Gamepad): The gamepad to use.

    """

    can_controller: CANController
    speed_controller: SpeedController

    cam_left: VideoStream
    cam_center: VideoStream
    cam_right: VideoStream

    lane_assist: LaneAssist
    detector: ObjectDetector

    gamepad: Gamepad

    def __init__(self) -> None:
        """Initialize the kart."""
        self.init_autonomous_driving()
        self.init_manual_driving()

    def start(self) -> "Kart":
        """Start the kart.

        This will start the kart and all its components.
        But it will only enable manual driving.
        """
        self.start_manual_driving()
        self.start_autonomous_driving()

        return self

    def init_manual_driving(self) -> None:
        """Initialize the kart for manual driving."""
        calibration_data = CalibrationData.load(config.calibration.calibration_file)
        self.__init_cameras()
        self.__init__kart_control(calibration_data)
        self.__init_gamepad_driving()

    def init_autonomous_driving(self) -> None:
        """Initialize the kart for autonomous driving."""
        calibration_data = CalibrationData.load(config.calibration.calibration_file)
        self.telemetry_server = TelemetryServer()

        self.__init_cameras()
        self.__init__kart_control(calibration_data)
        self.__init_object_detector(calibration_data)

    def start_autonomous_driving(self) -> None:
        """Start the autonomous driving."""
        self.cam_left.start()
        self.cam_center.start()
        self.cam_right.start()

        self.lane_assist.start()
        self.detector.start()
        self.telemetry_server.start()

    def start_manual_driving(self) -> None:
        """Start the manual driving."""
        self.gamepad_driving.start()
        self.gamepad.start()

    def __init__kart_control(self, calibration: CalibrationData) -> None:
        self.can_controller = CANController(get_can_bus())
        self.speed_controller = SpeedController(self.can_controller)

        path_follower = PathFollower(
            config.lane_assist.line_following.pid.kp,
            config.lane_assist.line_following.pid.ki,
            config.lane_assist.line_following.pid.kd,
            look_ahead_distance=config.lane_assist.line_following.look_ahead_distance,
            max_steering_range=config.lane_assist.line_following.max_steering_range
        )

        image_gen = td_stitched_image_generator(
            calibration,
            self.cam_left,
            self.cam_center,
            self.cam_right,
            self.telemetry_server
        )

        stopline = StopLineAssist(self.speed_controller, calibration),
        self.lane_assist = LaneAssist(
            image_gen,
            stopline,
            path_follower,
            self.speed_controller,
            telemetry=self.telemetry_server,
            calibration=calibration
        )

    def __init_cameras(self) -> None:
        self.cam_left = VideoStream(config.camera_ids.left)
        self.cam_center = VideoStream(config.camera_ids.center)
        self.cam_right = VideoStream(config.camera_ids.right)

    def __init_object_detector(self, calibration: CalibrationData) -> None:
        controller = ObjectController(calibration, self.lane_assist, self.speed_controller)
        controller.add_handler(PedestrianHandler(controller))
        controller.add_handler(SpeedLimitHandler(controller))
        controller.add_handler(TrafficLightHandler(controller))

        lidar = Lidar.safe_init()
        if lidar is not None:
            controller.add_handler(OvertakeHandler(controller, lidar))

        self.detector = ObjectDetector.from_model(config.object_detection.model_path, controller,
                                                  config.camera_ids.center)

    def __init_gamepad_driving(self) -> None:
        self.gamepad = Gamepad()
        self.gamepad_driving = BasicControllerDriving(self.gamepad, self.can_controller)

        self.gamepad.add_listener(GamepadButton.START, EventType.LONG_PRESS, self.__toggle)

    def __toggle(self, *_args: Any, **_kwargs: Any) -> None:
        """Toggle the controller."""
        self.gamepad.vibrate()
        time.sleep(3)

        self.gamepad.vibrate()
        self.gamepad_driving.toggle()
        self.lane_assist.toggle()
