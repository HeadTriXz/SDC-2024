import can
import logging
import os

from config import config
from driving.can_controller import CANController
from driving.speed_controller import SpeedController, SpeedControllerState
from object_recognition.handlers.pedestrian_handler import PedestrianHandler
from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
from object_recognition.handlers.traffic_light_handler import TrafficLightHandler
from object_recognition.object_controller import ObjectController
from object_recognition.object_detector import ObjectDetector


def initialize_can() -> can.Bus:
    """Initialize the can bus."""
    try:
        os.system("ip link set can0 type can bitrate 500000")
        os.system("ip link set can0 up")

        return can.Bus(interface="socketcan", channel="can0", bitrate=500000)
    except Exception:
        logging.warning("Failed to initialize the CAN bus. Using the virtual CAN bus instead.")
        return can.Bus(interface="virtual", channel="vcan0")


if __name__ == "__main__":
    can_bus = initialize_can()
    can_controller = CANController(can_bus)
    speed_controller = SpeedController(can_controller)
    speed_controller.state = SpeedControllerState.DRIVING

    controller = ObjectController(speed_controller)
    controller.add_handler(PedestrianHandler(controller))
    controller.add_handler(SpeedLimitHandler(controller))
    controller.add_handler(TrafficLightHandler(controller))

    detector = ObjectDetector.from_model(config.object_detection.model_path, controller, 0)
    can_controller.start()
    speed_controller.start()
    detector.start()

    while True:
        pass
