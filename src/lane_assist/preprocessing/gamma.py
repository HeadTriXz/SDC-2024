import cv2
import numpy as np


def adjust_gamma(image: np.ndarray, gamma: float = 1.0) -> np.ndarray:
    """Adjust the gamma of the image.

    :param image: The image to adjust.
    :param gamma: The gamma value to adjust the image.
    :return: The adjusted image.
    """
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(image, table)
