import sys
from os import system

import can

from common.camera_stream.stream import VideoStream
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
    cam1 = VideoStream(0)
    cam2 = VideoStream(4)
    cam3 = VideoStream(2)

    cam1.start()
    cam2.start()
    cam3.start()


    # connect to can bus
    bus = get_can_real_or_virtual()
    can_controller = CANController(bus)

    # configure a lane follower
    line_following = LineFollowing(0.1, 0.01, 0.05, look_ahead_distance=10)

    # start the lane assist
    lane_assist_thread = lane_assist(cam1, cam2, cam3, line_following, can_controller)

    # start the webserver
    start_telemetry(line_following)
