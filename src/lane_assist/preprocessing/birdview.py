import cv2
import numpy as np

from lane_assist.preprocessing.utils.corners import get_transformed_corners


def warp_image(image: np.ndarray, matrix: np.ndarray, max_height: int = None) -> np.ndarray:
    """Warp an image using a perspective matrix.

    :param image: The image to warp.
    :param matrix: The perspective matrix.
    :param max_height: The maximum height of the new image.
    :return: The warped image and the amount cropped from the top.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, (image.shape[0], image.shape[1]))

    width = int(max_x - min_x)
    height = int(max_y - min_y)
    if max_height is not None and height > max_height:
        height = max_height
        min_y = max_y - height

    adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
    adjusted_matrix = np.dot(adjusted_matrix, matrix)

    return cv2.warpPerspective(image, adjusted_matrix, (width, height), flags=cv2.INTER_NEAREST)
