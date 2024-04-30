import cv2
import numpy as np


class GammaAdjuster:
    """A class to adjust the gamma of an image.

    Attributes
    ----------
        tables: The lookup tables for the gamma values.

    """

    tables: dict[float, np.ndarray]

    def __init__(self) -> None:
        """Initialize the gamma adjuster."""
        self.tables = {}

    def adjust(self, image: np.ndarray, gamma: float = 1.0) -> np.ndarray:
        """Adjust the gamma of the image.

        :param image: The image to adjust.
        :param gamma: The gamma value to adjust the image.
        :return: The adjusted image.
        """
        if gamma == 1.0:
            return image

        if gamma not in self.tables:
            inv_gamma = 1.0 / gamma
            table = (np.arange(256) / 255.0) ** inv_gamma * 255

            self.tables[gamma] = table.astype(np.uint8)

        return cv2.LUT(image, self.tables[gamma])
