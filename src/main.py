import sys
import threading
from os import system

import can

from common.camera_stream.stream import VideoStream
from common.constants import Gear
from globals import GLOBALS
from kart_control.can_controller import CANController
from kart_control.speed_controller import SpeedController
from lane_assist import LaneLynx, LineFollowing
from lane_assist.image_manipulation import td_stitched_image_generator
from telemetry import start_telemetry


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

    line_following = LineFollowing(0.1, 0.01, 0.05, look_ahead_distance=10)

    telem_thread = threading.Thread(target=start_telemetry, args=(line_following,), daemon=True)
    telem_thread.start()
    #
    speed_controller = SpeedController(can_controller)
    speed_controller.gear = Gear.DRIVE

    lynx = LaneLynx(
        td_stitched_image_generator(cam1, cam2, cam3),
        line_following,
        speed_controller,
        adjust_speed=lambda _: GLOBALS["SET_SPEED"],
    )
    lynx.start()
