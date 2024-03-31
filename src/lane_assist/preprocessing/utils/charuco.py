import numpy as np

from lane_assist.preprocessing.utils.corners import find_largest_rectangle
from lane_assist.preprocessing.utils.other import euclidean_distance


def find_corners(grid: np.ndarray) -> tuple[np.ndarray, tuple[int, int]]:
    """Find the ChArUco corners in a grid.

    :param grid: The grid to find the corners in.
    :return: The corners of the ChArUco board.
    """
    binary_matrix = np.any(grid, axis=2).astype(np.uint8)

    # Find the largest rectangle in the binary matrix
    corner_indices = find_largest_rectangle(binary_matrix)
    if corner_indices is None:
        raise ValueError("No rectangle found in the ChArUco board")

    rect_w = int(euclidean_distance(corner_indices[0], corner_indices[1]))
    rect_h = int(euclidean_distance(corner_indices[1], corner_indices[2]))
    shape = (rect_w, rect_h)

    return grid[corner_indices[:, 0], corner_indices[:, 1]], shape

