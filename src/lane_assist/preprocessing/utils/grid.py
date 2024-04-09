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


def get_dst_points(length: float) -> np.ndarray:
    """Calculate the destination points for the image.

    :param length: The length of a single square.
    :return: The destination points for the image.
    """
    w, h = np.subtract(get_board_shape(), 1)
    points = np.zeros((h * w, 2), dtype=np.float32)

    for i in range(h):
        for j in range(w):
            x = j * length
            y = i * length

            points[i * w + j] = [x, y]

    return points


def merge_grids(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Combine two grids.

    :param a: The first grid.
    :param b: The second grid.
    :return: The combined grid.
    """
    for row in range(b.shape[0]):
        for col in range(b.shape[1]):
            if not np.any(b[row, col]):
                continue

            a[row, col] = b[row, col]

    return a
