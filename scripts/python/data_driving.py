import cv2
import sys
import time

from datetime import datetime
from pathlib import Path

from src.config import config
from src.constants import CameraResolution
from src.driving.can import get_can_bus, CANController
from src.driving.gamepad import Gamepad
from src.driving.modes import ManualDriving
from src.utils.video_stream import VideoStream


def start_collecting() -> None:
    """Start collecting data."""
    folder_name = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    folder = Path("./data/images/" + folder_name)
    folder.mkdir(parents=True, exist_ok=True)

    print("Initializing...", file=sys.stderr)  # noqa: T201

    center_cam = VideoStream(config["camera_ids"]["center"], resolution=CameraResolution.HD)
    center_cam.start()

    if not center_cam.has_next():
        raise ValueError("Could not capture images from cameras")

    print("Collecting data...", file=sys.stderr)  # noqa: T201

    try:
        while True:
            timestamp = time.time_ns() // 1000000
            center_image = center_cam.next()

            cv2.imwrite(str(folder / f"{timestamp}.jpg"), center_image)
    except KeyboardInterrupt:
        pass

    print("Stopping...", file=sys.stderr)  # noqa: T201

    center_cam.stop()


def start_driving() -> None:
    """Start driving the kart."""
    bus = get_can_bus()
    can = CANController(bus)
    can.start()

    gamepad = Gamepad()
    gamepad.start()

    controller = ManualDriving(gamepad, can)
    controller.start()


def main() -> None:
    """Start the data acquisition."""
    start_driving()
    start_collecting()


if __name__ == "__main__":
    main()
