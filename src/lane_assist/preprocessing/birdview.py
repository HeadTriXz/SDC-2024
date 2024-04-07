import cv2
import numpy as np

from lane_assist.preprocessing.calibrate import CameraCalibrator


def warp_image(calibrator: CameraCalibrator, image: np.ndarray, idx: int) -> np.ndarray:
    """Warp an image using a perspective matrix.

    :param calibrator: The camera calibrator.
    :param image: The image to warp.
    :param idx: The camera to select the configuration for.
    :return: The warped image and the amount cropped from the top.
    """
    return cv2.warpPerspective(image, calibrator.matrices[idx], calibrator.shapes[idx], flags=cv2.INTER_NEAREST)
