import can
import logging
import sys
import threading

from os import system
from config import config
from constants import Gear
from driving.can_controller import CANController
from driving.speed_controller import SpeedController, SpeedControllerState
from object_recognition.handlers.pedestrian_handler import PedestrianHandler
from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from object_recognition.object_controller import ObjectController
from object_recognition.object_detector import ObjectDetector
from lane_assist.helpers import td_stitched_image_generator
from lane_assist.lane_assist import LaneAssist
from lane_assist.line_following.path_follower import PathFollower
from telemetry import start_telemetry
from utils.video_stream import VideoStream


def initialize_can() -> can.Bus:
    """Initialize the can bus."""
    system("ip link set can0 type can bitrate 500000")
    system("ip link set can0 up")

    return can.Bus(interface="socketcan", channel="can0", bitrate=500000)


def get_can_real_or_virtual() -> can.Bus:
    """Get the can bus."""
    if sys.platform == "linux":
        return initialize_can()

    return can.Bus(interface="virtual", channel="vcan0")


def main() -> None:
    """Start the main loop."""
    # Load cameras
    cam_left = VideoStream(config.camera_ids.left)
    cam_center = VideoStream(config.camera_ids.center)
    cam_right = VideoStream(config.camera_ids.right)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    # Connect to CAN bus
    bus = get_can_real_or_virtual()
    can_controller = CANController(bus)
    speed_controller = SpeedController(can_controller)

    # Initialize the speed controller
    speed_controller.gear = Gear.DRIVE
    speed_controller.state = SpeedControllerState.WAITING_TO_STOP
    speed_controller.max_speed = 50

    # Initialize the path follower
    path_follower = PathFollower(0.1, 0.01, 0.05, look_ahead_distance=10)
    path_follower.max_steering_range = 30.0

    # Start telemetry
    telem_thread = threading.Thread(target=start_telemetry, args=(path_follower,), daemon=True)
    telem_thread.start()

    # Initialize the object controller
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


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
