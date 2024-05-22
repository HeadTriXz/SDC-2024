import cv2
import sys
import time

from datetime import datetime
from pathlib import Path

from src.config import config
from src.constants import CameraResolution
from src.utils.video_stream import VideoStream


def main() -> None:
    """Start collecting data."""
    folder_name = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    folder = Path("../data/images/" + folder_name)
    folder.mkdir(parents=True, exist_ok=True)

    print("Initializing...", file=sys.stderr)  # noqa: T201

    left_cam = VideoStream(config["camera_ids"]["left"], resolution=CameraResolution.FHD)
    center_cam = VideoStream(config["camera_ids"]["center"], resolution=CameraResolution.FHD)
    right_cam = VideoStream(config["camera_ids"]["right"], resolution=CameraResolution.FHD)

    left_cam.start()
    center_cam.start()
    right_cam.start()

    if not left_cam.has_next() or not center_cam.has_next() or not right_cam.has_next():
        raise ValueError("Could not capture images from cameras")

    print("Collecting data...", file=sys.stderr)  # noqa: T201

    try:
        while True:
            timestamp = time.time_ns() // 1000000

            left_image = left_cam.next()
            center_image = center_cam.next()
            right_image = right_cam.next()

            cv2.imwrite(str(folder / f"{timestamp}_left.jpg"), left_image)
            cv2.imwrite(str(folder / f"{timestamp}_center.jpg"), center_image)
            cv2.imwrite(str(folder / f"{timestamp}_right.jpg"), right_image)
    except KeyboardInterrupt:
        pass

    print("Stopping...", file=sys.stderr)  # noqa: T201

    left_cam.stop()
    center_cam.stop()
    right_cam.stop()


if __name__ == "__main__":
    main()
