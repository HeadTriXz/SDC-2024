from typing import Generator

import cv2
import numpy as np

from globals import GLOBALS
from lane_assist import adjust_gamma, stitch_images, topdown
from utils.video_stream import VideoStream


def td_stitched_image_generator(
    left_cam: VideoStream, center_cam: VideoStream, right_cam: VideoStream
) -> Generator[np.ndarray, None, None]:
    """Generate a picture from the cameras.

    This function will take a picture from the cameras and return the topdown image.
    It will also convert to grayscale.
    This is a generator function, so we can use it in a for loop.
    This will make it easier to use in the lane assist.

    Parameters
    ----------
    :param left_cam: The left camera.
    :param center_cam: The center camera.
    :param right_cam: The right camera.

    """

    def __generator() -> Generator[np.ndarray, None, None]:
        while left_cam.has_next() and center_cam.has_next() and right_cam.has_next():
            left_image = left_cam.next()
            center_image = center_cam.next()
            right_image = right_cam.next()

            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            center_image = cv2.cvtColor(center_image, cv2.COLOR_BGR2GRAY)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            if GLOBALS["GAMMA"]["ADJUST"]:
                left_image = adjust_gamma(left_image, GLOBALS["GAMMA"]["LEFT"])
                center_image = adjust_gamma(center_image, GLOBALS["GAMMA"]["CENTER"])
                right_image = adjust_gamma(right_image, GLOBALS["GAMMA"]["RIGHT"])

            stitched_image = stitch_images(left_image, center_image, right_image)
            topdown_image = topdown(stitched_image)

            yield topdown_image

    return __generator()
