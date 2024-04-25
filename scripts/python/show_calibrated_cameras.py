import cv2
import logging
import numpy as np
import requests

from config import config
from constants import CameraResolution
from pathlib import Path
from utils.calibration_data import CalibrationData
from utils.video_stream import VideoStream

WEBHOOK_URL = "https://discord.com/api/webhooks/1233031145357049866/a4-YT4xhy-pxlMx7-8XI2cvzlrWEXQlyjVvl0QKVaPbBvJv80jBJ4ThuDEmRIbyuev7O"


def send_discord_image(image: np.ndarray) -> None:
    """Send an image to a Discord webhook.

    :param image: The image to send.
    """
    retval, buffer = cv2.imencode(".png", image)
    requests.post(WEBHOOK_URL, files={"file.png": buffer.tobytes()})


def send_discord_calibration() -> None:
    left_cam = VideoStream(config.camera_ids.left, resolution=CameraResolution.NHD)
    center_cam = VideoStream(config.camera_ids.center, resolution=CameraResolution.HD)
    right_cam = VideoStream(config.camera_ids.right, resolution=CameraResolution.NHD)

    left_cam.start()
    center_cam.start()
    right_cam.start()

    if not left_cam.has_next() or not center_cam.has_next() or not right_cam.has_next():
        raise ValueError("Could not capture images from cameras")

    left_image = left_cam.next()
    center_image = center_cam.next()
    right_image = right_cam.next()

    left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    center_image = cv2.cvtColor(center_image, cv2.COLOR_BGR2GRAY)
    right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

    calibration_path = Path(config.calibration.calibration_file)
    if not calibration_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

    calibration_data = CalibrationData.load(calibration_path)
    topdown = calibration_data.transform([left_image, center_image, right_image])

    try:
        send_discord_image(topdown)
    except Exception as e:
        logging.warning("Failed to send image to Discord: %s", e)

    left_cam.stop()
    center_cam.stop()
    right_cam.stop()


if __name__ == "__main__":
    send_discord_calibration()
