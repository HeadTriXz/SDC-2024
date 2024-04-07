from typing import Callable, Generator

import cv2
import numpy as np

from config import config
from lane_assist.preprocessing.birdview import warp_image
from lane_assist.preprocessing.calibrate import CameraCalibrator
from lane_assist.preprocessing.gamma import adjust_gamma
from lane_assist.preprocessing.stitching import stitch_images
from utils.video_stream import VideoStream


def td_stitched_image_generator(
        calibrator: CameraCalibrator,
        left_cam: VideoStream,
        center_cam: VideoStream,
        right_cam: VideoStream
) -> Callable[[], Generator[np.ndarray, None, None]]:
    """Generate a picture from the cameras.

    This function will take a picture from the cameras and return the topdown image.
    It will also convert to grayscale.
    This is a generator function, so we can use it in a for loop.
    This will make it easier to use in the lane assist.

    :param calibrator: The loaded camera calibrator.
    :param left_cam: The left camera.
    :param center_cam: The center camera.
    :param right_cam: The right camera.
    """

    def __generator() -> Generator[np.ndarray, None, None]:
        """Generate a topdown image from the cameras."""
        while left_cam.has_next() and center_cam.has_next() and right_cam.has_next():
            left_image = left_cam.next()
            center_image = center_cam.next()
            right_image = right_cam.next()

            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            center_image = cv2.cvtColor(center_image, cv2.COLOR_BGR2GRAY)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            if config.image_manipulation.gamma.adjust:
                left_image = adjust_gamma(left_image, config.image_manipulation.gamma.left)
                center_image = adjust_gamma(center_image, config.image_manipulation.gamma.center)
                right_image = adjust_gamma(right_image, config.image_manipulation.gamma.right)

            warped_left = warp_image(calibrator, left_image, idx=0)
            warped_right = warp_image(calibrator, right_image, idx=2)

            stitched = np.zeros(calibrator.stitched_shape[::-1], dtype=np.uint8)
            stitched = stitch_images(stitched, warped_right, calibrator.offsets[2])
            stitched = stitch_images(stitched, warped_left, calibrator.offsets[0])
            stitched = stitch_images(stitched, center_image, calibrator.offsets[1])

            yield cv2.warpPerspective(
                stitched,
                calibrator.topdown_matrix,
                calibrator.output_shape,
                flags=cv2.INTER_NEAREST
            )

    return __generator
