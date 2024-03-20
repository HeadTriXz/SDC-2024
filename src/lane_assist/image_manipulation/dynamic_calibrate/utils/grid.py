import cv2
import numpy as np

from lane_assist.image_manipulation.dynamic_calibrate.utils.other import get_board_shape


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


def get_src_grid(detector: cv2.aruco.CharucoDetector, image: np.ndarray) -> np.ndarray:
    """Get the source grid for the image.

    :param detector: The ChArUco detector.
    :param image: The image to get the grid for.
    :return: The source grid for the image.
    """
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
    if charuco_corners is None or len(charuco_corners) < 4:
        raise ValueError("The ChArUco board was not detected")

    return corners_to_grid(charuco_corners, charuco_ids, get_board_shape())


def get_dst_grid(h_change: float, v_change: float) -> np.ndarray:
    """Calculate the destination grid for the image.

    :param h_change: The horizontal change per square.
    :param v_change: The vertical change per square.
    :return: The destination grid for the image.
    """
    w, h = np.subtract(get_board_shape(), 1)
    dst_grid = np.zeros((h, w, 2), dtype=np.float32)

    for i in range(h):
        for j in range(w):
            x = j * v_change + i * h_change
            y = (w - j) * h_change + i * v_change

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
