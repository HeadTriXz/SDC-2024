from src.calibration.data import CalibrationData
from src.config import config
from src.constants import CameraResolution, Gear
from src.driving.can import CANController
from src.driving.modes import DrivingMode
from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.lane_assist.preprocessing.generator import td_stitched_image_generator
from src.lane_assist.stop_line_assist import StopLineAssist
from src.object_recognition.handlers.overtake_handler import OvertakeHandler
from src.object_recognition.handlers.parking_handler import ParkingHandler
from src.object_recognition.handlers.pedestrian_handler import PedestrianHandler
from src.object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from src.object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from src.object_recognition.object_controller import ObjectController
from src.object_recognition.object_detector import ObjectDetector
from src.telemetry.app import TelemetryServer
from src.utils.lidar import Lidar
from src.utils.video_stream import VideoStream


class AutonomousDriving(DrivingMode):
    """The autonomous driving mode.

    Attributes
    ----------
        cam_left (VideoStream): The left camera stream.
        cam_center (VideoStream): The center camera stream.
        cam_right (VideoStream): The right camera stream.
        detector (ObjectDetector): The object detector.
        lane_assist (LaneAssist): The lane assist.
        speed_controller (SpeedController): The speed controller.
        telemetry (TelemetryServer): The telemetry server.

    """

    cam_left: VideoStream
    cam_center: VideoStream
    cam_right: VideoStream
    detector: ObjectDetector
    lane_assist: LaneAssist
    speed_controller: SpeedController
    telemetry: TelemetryServer

    def __init__(self, can_controller: CANController) -> None:
        """Initialize the autonomous driving system.

        :param can_controller: The CAN controller to use.
        """
        calibration = CalibrationData.load(config["calibration"]["calibration_file"])

        self.cam_left = VideoStream(config["camera_ids"]["left"], resolution=CameraResolution.NHD)
        self.cam_center = VideoStream(config["camera_ids"]["center"], resolution=CameraResolution.HD)
        self.cam_right = VideoStream(config["camera_ids"]["right"], resolution=CameraResolution.NHD)

        self.telemetry = TelemetryServer()
        self.speed_controller = SpeedController(can_controller)

        self.__init_lane_assist(calibration)
        self.__init_object_detection(calibration)

    def start(self) -> None:
        """Start the autonomous driving system."""
        self.cam_left.start()
        self.cam_center.start()
        self.cam_right.start()

        self.speed_controller.start()
        self.speed_controller.gear = Gear.DRIVE
        self.speed_controller.state = SpeedControllerState.DRIVING

        self.detector.start()
        self.telemetry.start()
        self.lane_assist.start()

    def toggle(self) -> None:
        """Toggle the autonomous driving."""
        self.lane_assist.toggle()
        self.speed_controller.toggle()

    def __init_lane_assist(self, calibration: CalibrationData) -> None:
        """Initialize the lane assist system.

        :param calibration: The calibration data to use.
        """
        generator = td_stitched_image_generator(
            calibration,
            self.cam_left,
            self.cam_center,
            self.cam_right,
            self.telemetry
        )

        stop_line_assist = StopLineAssist(self.speed_controller, calibration)
        self.lane_assist = LaneAssist(
            generator,
            stop_line_assist,
            self.speed_controller,
            self.telemetry,
            calibration
        )

    def __init_object_detection(self, calibration: CalibrationData) -> None:
        """Initialize the object detection system.

        :param calibration: The calibration data to use.
        """
        object_controller = ObjectController(calibration, self.lane_assist, self.speed_controller)
        object_controller.add_handler(PedestrianHandler(object_controller))
        object_controller.add_handler(SpeedLimitHandler(object_controller))
        object_controller.add_handler(TrafficLightHandler(object_controller))

        lidar = Lidar.safe_init()
        if lidar is not None:
            object_controller.add_handler(OvertakeHandler(object_controller, lidar))
            object_controller.add_handler(ParkingHandler(object_controller))

        self.detector = ObjectDetector.from_model(
            config["object_detection"]["model_path"], object_controller, config["camera_ids"]["center"]
        )
