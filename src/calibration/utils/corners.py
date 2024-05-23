import cv2
import numpy as np

from src.utils.other import get_border_of_points


def find_largest_rectangle(matrix: np.ndarray) -> np.ndarray | None:
    """Find the largest rectangle in a binary matrix.

    :param matrix: The binary matrix.
    :return: The corners of the largest rectangle (tl, tr, br, bl).
    """
    if not np.any(matrix):
        return None

    max_area = 0
    top_left = (0, 0)
    bottom_right = (0, 0)

    rows, cols = matrix.shape[:2]

    for i in range(rows):
        for j in range(cols):
            if matrix[i, j] == 0:
                continue

            for x in range(i, rows):
                for y in range(j, cols):
                    if matrix[x][y] == 1 and matrix[i][y] == 1 and matrix[x][j] == 1:
                        area = (x - i + 1) * (y - j + 1)
                        if area > max_area:
                            max_area = area
                            top_left = (i, j)
                            bottom_right = (x, y)

    top_right = (top_left[0], bottom_right[1])
    bottom_left = (bottom_right[0], top_left[1])

    return np.array([top_left, top_right, bottom_right, bottom_left])


def get_transformed_corners(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """Get the transformed corners of the image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :return: The transformed corners of the image.
    """
    h, w = shape
    src_points = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.float32)
    dst_points = cv2.perspectiveTransform(src_points, matrix)

    return get_border_of_points(dst_points[:, 0])
