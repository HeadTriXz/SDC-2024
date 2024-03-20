import cv2
import numpy as np

from typing import Optional


def find_largest_rectangle(matrix: np.ndarray) -> Optional[np.ndarray]:
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

    min_x = np.amin(dst_points[:, 0, 0])
    min_y = np.amin(dst_points[:, 0, 1])
    max_x = np.amax(dst_points[:, 0, 0])
    max_y = np.amax(dst_points[:, 0, 1])
    return int(min_x), int(min_y), int(max_x), int(max_y)


def get_dst_corners(
        src_corners: np.ndarray,
        h_change: float,
        v_change: float,
        shape: tuple[int, int],
        scale_factor: float = 1.0
) -> np.ndarray:
    """Calculate the destination corners of the ChArUco board.

    :param src_corners: The original points of the ChArUco board.
    :param h_change: The horizontal change per square.
    :param v_change: The vertical change per square.
    :param shape: The shape of the rectangle.
    :param scale_factor: The scale factor for the perspective matrix.
    :return: The destination corners of the ChArUco board.
    """
    w, h = shape
    cx, cy = np.mean(src_corners, axis=0)

    new_w = int(w * h_change * scale_factor)
    new_h = int(h * (h_change + v_change) * scale_factor)

    return np.array([
        [cx - new_h / 2, cy + new_w / 2],
        [cx - new_h / 2, cy - new_w / 2],
        [cx + new_h / 2, cy - new_w / 2],
        [cx + new_h / 2, cy + new_w / 2]
    ])
