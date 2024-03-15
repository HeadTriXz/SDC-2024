from typing import Optional

import cv2
import numpy as np

Coordinate = tuple[int, int]
Corners = tuple[Coordinate, Coordinate, Coordinate, Coordinate]


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


def euclidean_distance(p1: np.ndarray | Coordinate, p2: np.ndarray | Coordinate) -> float:
    """Calculate the Euclidean distance between two points.

    :param p1: The first point.
    :param p2: The second point.
    :return: The Euclidean distance between the two points.
    """
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def corners_to_grid(corners: np.ndarray, ids: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Convert corners and ids to a grid.

    :param corners: An array of corners.
    :param ids: An array of ids for each corner.
    :param shape: The shape of the board (w, h).
    :return: The grid of corners.
    """
    w, h = np.subtract(shape, 1)
    grid = np.zeros((h, w, 2), dtype=np.float32)

    for corner, corner_id in zip(corners[:, 0], ids[:, 0]):
        col = corner_id % w
        row = corner_id // w

        grid[row, col] = corner

    return grid


def adjust_perspective(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[np.ndarray, int, int]:
    """Adjust the perspective matrix to fit within the new image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the old image.
    :return: The adjusted perspective matrix and the new shape of the image.
    """
    h, w = shape
    src_points = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32).reshape(-1, 1, 2)
    dst_points = cv2.perspectiveTransform(src_points, matrix)

    min_x = np.amin(dst_points[:, 0, 0])
    min_y = np.amin(dst_points[:, 0, 1])
    max_x = np.amax(dst_points[:, 0, 0])
    max_y = np.amax(dst_points[:, 0, 1])

    new_w = int(max_x - min_x)
    new_h = int(max_y - min_y)

    adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
    adjusted_matrix = np.dot(adjusted_matrix, matrix)

    return adjusted_matrix, new_w, new_h


def scale_matrix(matrix: np.ndarray, scale_factor: float) -> np.ndarray:
    """Scale the perspective matrix.

    :param matrix: The perspective matrix.
    :param scale_factor: The scale factor.
    :return: The scaled perspective matrix.
    """
    scaled_matrix = np.array([[scale_factor, 0, 0], [0, scale_factor, 0], [0, 0, 1]], dtype=np.float32)
    return np.dot(scaled_matrix, matrix)
