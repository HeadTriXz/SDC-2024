
from config import config
from constants import Gear
from driving.can_controller import CANController
from driving.can_controller.can_bus import get_can_interface
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.helpers import td_stitched_image_generator
from lane_assist.lane_assist import LaneAssist
from lane_assist.line_following.path_follower import PathFollower
from object_recognition.handlers.pedestrian_handler import PedestrianHandler
from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from object_recognition.object_controller import ObjectController
from object_recognition.object_detector import ObjectDetector
from utils.video_stream import VideoStream
from telemetry.webapp.telemetry_server import TelemetryServer


def main() -> None:
    """Start the main loop."""
    cam_left = VideoStream(config.camera_ids.left)
    cam_center = VideoStream(config.camera_ids.center)
    cam_right = VideoStream(config.camera_ids.right)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    # Connect to CAN bus
    bus = get_can_interface()
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

    # Initialize the lane assist
    lane_assist = LaneAssist(
        td_stitched_image_generator(cam_left, cam_center, cam_right),
        path_follower,
        speed_controller,
        adjust_speed=lambda _: 1,
    )

    # Initialize the object detector
    detector = ObjectDetector.from_model(config.object_detection.model_path, controller, 0)

    # Start the system
    can_controller.start()
    speed_controller.start()
    detector.start()
    lane_assist.start()

    server = TelemetryServer()
    server.start()
    input("Press Enter to stop...")


if __name__ == "__main__":
    """The main function."""
    from simulator import main as smain

    smain()
