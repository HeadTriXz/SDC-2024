import sys
from os import system

import can
import cv2

from kart_control.can_controller import CANController
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


if __name__ == "__main__":
    # load cameras
    cam1 = cv2.VideoCapture(0)
    cam2 = cv2.VideoCapture(1)
    cam3 = cv2.VideoCapture(2)

    # connect to can bus
    bus = get_can_real_or_virtual()
    can_controller = CANController(bus)

    # configure a lane follower
    line_following = LineFollowing(0.1, 0.01, 0.05, look_ahead_distance=10)

    # start the lane assist
    lane_assist_thread = lane_assist(cam1, cam2, cam3, line_following, can_controller)

    # start the webserver
    start_telemetry(line_following)
