import cv2
import numpy as np

from lane_assist.preprocessing.utils.other import get_board_shape


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


def get_dst_grid(length: float, angle: float) -> np.ndarray:
    """Calculate the destination grid for the image.

    :param length: The length of a single square.
    :param angle: The angle of the board in radians.
    :return: The destination grid for the image.
    """
    w, h = np.subtract(get_board_shape(), 1)
    dst_grid = np.zeros((h, w, 2), dtype=np.float32)

    for i in range(h):
        for j in range(w):
            x = j * length * np.cos(angle) - i * length * np.sin(angle)
            y = j * length * np.sin(angle) + i * length * np.cos(angle)

            dst_grid[i, j] = [x, y]

    return dst_grid


def crop_grid(grid: np.ndarray, amount: int) -> np.ndarray:
    """Crop a grid by a certain amount.

    :param grid: The grid to crop.
    :param amount: The amount to crop the grid by.
    :return: The cropped grid.
    """
    new_grid = np.zeros_like(grid)
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if not np.any(grid[row, col]):
                continue

            new_grid[row, col] = grid[row, col] - [0, amount]

    return new_grid
