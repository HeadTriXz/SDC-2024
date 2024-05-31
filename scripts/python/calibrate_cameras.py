import cv2
import logging
import numpy as np

from pathlib import Path

from scripts.python.show_calibrated_cameras import send_discord_calibration
from src.calibration.calibrate import CameraCalibrator
from src.config import config
from src.constants import CameraResolution
from src.utils.video_stream import VideoStream


def calibrate_images(images: list[np.ndarray]) -> None:
    """Calibrate the cameras using the images.

    :param images The images to use for the calibration.
    """
    calibrator = CameraCalibrator(images, input_shape=CameraResolution.NHD)
    calibrator.calibrate()

    save_dir = Path(config["calibration"]["save_dir"])
    history_file = calibrator.save(save_dir)

    logging.info("Saved the calibration results to %s. Output shape: %s", history_file, calibrator.output_shape)

    # Save the used images to the images dir.
    images_dir = save_dir / "images" / history_file.stem
    images_dir.mkdir(exist_ok=True, parents=True)

    cv2.imwrite(str(images_dir / "left.png"), images[0])
    cv2.imwrite(str(images_dir / "center.png"), images[1])
    cv2.imwrite(str(images_dir / "right.png"), images[2])

    # Send an example to Discord.
    send_discord_calibration()


def calibrate_cameras() -> None:
    """A script to calibrate the cameras."""
    if len({config["camera_ids"]["left"], config["camera_ids"]["center"], config["camera_ids"]["right"]}) < 3:
        raise ValueError("Not all camera ids are unique, calibration may not work as expected")

    # Initialize the camera streams.
    cam_left = VideoStream(config["camera_ids"]["left"], resolution=CameraResolution.FHD)
    cam_center = VideoStream(config["camera_ids"]["center"], resolution=CameraResolution.FHD)
    cam_right = VideoStream(config["camera_ids"]["right"], resolution=CameraResolution.FHD)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    if not cam_left.has_next() or not cam_center.has_next() or not cam_right.has_next():
        raise ValueError("Could not capture images from cameras")

    # Calibrate the cameras.
    left_image = cam_left.next()
    center_image = cam_center.next()
    right_image = cam_right.next()

    calibrate_images([left_image, center_image, right_image])

    # Stop the camera streams.
    cam_left.stop()
    cam_center.stop()
    cam_right.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    calibrate_cameras()
