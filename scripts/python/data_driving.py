import argparse
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


def start_collecting(all_cameras: bool = False) -> None:
    """Start collecting data.

    :param all_cameras: Whether to use all available cameras (left, center, right).
    """
    folder_name = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    folder = Path("./data/images/" + folder_name)
    folder.mkdir(parents=True, exist_ok=True)

    print("Initializing...", file=sys.stderr)  # noqa: T201

    center_cam = VideoStream(config["camera_ids"]["center"], resolution=CameraResolution.HD)
    center_cam.start()

    left_cam = None
    right_cam = None
    if all_cameras:
        left_cam = VideoStream(config["camera_ids"]["left"], resolution=CameraResolution.HD)
        right_cam = VideoStream(config["camera_ids"]["right"], resolution=CameraResolution.HD)

        left_cam.start()
        right_cam.start()

    if not center_cam.has_next() or (all_cameras and (not left_cam.has_next() or not right_cam.has_next())):
        raise ValueError("Could not capture images from cameras")

    print("Collecting data...", file=sys.stderr)  # noqa: T201

    try:
        while True:
            timestamp = time.time_ns() // 1000000
            center_image = center_cam.next()

            cv2.imwrite(str(folder / f"{timestamp}_center.jpg"), center_image)

            if all_cameras:
                left_image = left_cam.next()
                right_image = right_cam.next()

                cv2.imwrite(str(folder / f"{timestamp}_left.jpg"), left_image)
                cv2.imwrite(str(folder / f"{timestamp}_right.jpg"), right_image)
    except KeyboardInterrupt:
        pass

    print("Stopping...", file=sys.stderr)  # noqa: T201

    center_cam.stop()
    if all_cameras:
        left_cam.stop()
        right_cam.stop()


def start_driving() -> None:
    """Start driving the kart."""
    bus = get_can_bus()
    can = CANController(bus)
    can.start()

    gamepad = Gamepad()
    gamepad.start()

    controller = ManualDriving(gamepad, can)
    controller.start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start collecting images while driving.")
    parser.add_argument("--all", action="store_true", help="Use all available cameras (left, center, right).")

    args = parser.parse_args()

    start_driving()
    start_collecting(args.all)
