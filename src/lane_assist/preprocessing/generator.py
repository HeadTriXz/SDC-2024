import cv2
import numpy as np

from typing import Callable, Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.lane_assist.preprocessing.gamma import GammaAdjuster
from src.telemetry.app import TelemetryServer
from src.utils.video_stream import VideoStream


def td_stitched_image_generator(
    calibration: CalibrationData,
    left_cam: VideoStream,
    center_cam: VideoStream,
    right_cam: VideoStream,
    telemetry: TelemetryServer
) -> Callable[[], Generator[np.ndarray, None, None]]:
    """Generate a picture from the cameras.

    This function will take a picture from the cameras and return the topdown image.
    It will also convert to grayscale.
    This is a generator function, so we can use it in a for loop.
    This will make it easier to use in the lane assist.

    :param calibration: The calibration data.
    :param left_cam: The left camera.
    :param center_cam: The center camera.
    :param right_cam: The right camera.
    :param telemetry: The telemetry server.
    """
    gamma_adjuster = GammaAdjuster()

    def __generator() -> Generator[np.ndarray, None, None]:
        """Generate a topdown image from the cameras."""
        while left_cam.has_next() and center_cam.has_next() and right_cam.has_next():
            left_image = left_cam.next()
            center_image = center_cam.next()
            right_image = right_cam.next()

            left_image = __transform_img(left_image)
            center_image = __transform_img(center_image)
            right_image = __transform_img(right_image)

            if config.image_manipulation.gamma.enabled:
                left_image = gamma_adjuster.adjust(left_image, config.image_manipulation.gamma.left)
                center_image = gamma_adjuster.adjust(center_image, config.image_manipulation.gamma.center)
                right_image = gamma_adjuster.adjust(right_image, config.image_manipulation.gamma.right)

            topdown = calibration.transform([left_image, center_image, right_image])

            thresholded = cv2.threshold(
                topdown, config.image_manipulation.white_threshold, 255, cv2.THRESH_BINARY
            )[1]

            # FIXME: remove telemetry
            if config.telemetry.enabled:
                telemetry.websocket_handler.send_image("left", left_image)
                telemetry.websocket_handler.send_image("center", center_image)
                telemetry.websocket_handler.send_image("right", right_image)
                telemetry.websocket_handler.send_image("topdown", topdown)

            yield thresholded

    return __generator


def __transform_img(img: np.ndarray) -> np.ndarray:
    """Convert the image to grayscale."""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
