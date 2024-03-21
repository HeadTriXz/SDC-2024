import sys
from os import system

import can

from common.camera_stream.stream import VideoStream
from common.constants import Gear
from globals import GLOBALS
from kart_control.can_controller import CANController
from kart_control.speed_controller import SpeedController
from lane_assist import LineFollowing, lane_assist
from telemetry import start as start_telemetry


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


def adust_speed(speed_controller: SpeedController) -> None:
    """Adjust the speed to the global set speed"""
    while True:
        speed_controller.max_speed = GLOBALS["SET_SPEED"]
        speed_controller.target_speed = GLOBALS["SET_SPEED"]


if __name__ == "__main__":
    # load cameras
    cam1 = VideoStream(0)
    cam2 = VideoStream(0)
    cam3 = VideoStream(0)

    cam1.start()
    cam2.start()
    cam3.start()

    # connect to can bus
    bus = get_can_real_or_virtual()
    can_controller = CANController(bus)
    # speed_controller = SpeedController(can_controller)
    # speed_controller.start()
    # speed_controller.gear = Gear.DRIVE

    # speed_controller_thread = threading.Thread(target=adust_speed, args=(speed_controller,), daemon=True)
    # speed_controller_thread.start()

    # configure a lane follower
    line_following = LineFollowing(0.1, 0.01, 0.05, look_ahead_distance=10)
    # start the lane assist
    lane_assist_thread = lane_assist(cam1, cam2, cam3, line_following, can_controller)

    # start the webserver
    start_telemetry(line_following)
