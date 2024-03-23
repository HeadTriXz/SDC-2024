import sys
import threading
from os import system

import can

from constants import Gear
from driving.can_controller import CANController
from driving.speed_controller import SpeedController
from globals import GLOBALS
from lane_assist import LaneLynx
from lane_assist.helpers import td_stitched_image_generator
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
    can_controller.start()

    path_follower = PathFollower(0.1, 0.01, 0.05, look_ahead_distance=10)

    telem_thread = threading.Thread(target=start_telemetry, args=(path_follower,), daemon=True)
    telem_thread.start()
    #
    speed_controller = SpeedController(can_controller)
    speed_controller.gear = Gear.DRIVE

    lynx = LaneLynx(
        td_stitched_image_generator(cam1, cam2, cam3),
        path_follower,
        speed_controller,
        adjust_speed=lambda _: GLOBALS["SET_SPEED"],
    )
    lynx.start()
