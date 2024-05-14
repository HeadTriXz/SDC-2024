import cv2
import numpy as np

from typing import Optional

from src.calibration.utils.corners import get_transformed_corners
from src.config import config


Coordinate = tuple[int, int] | np.ndarray


def euclidean_distance(p1: Coordinate, p2: Coordinate) -> float:
    """Calculate the Euclidean distance between two points.

    :param p1: The first point.
    :param p2: The second point.
    :return: The Euclidean distance between the two points.
    """
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def get_board_shape() -> tuple[int, int]:
    """Get the shape of the ChArUco board.

    :return: The shape of the ChArUco board.
    """
    return config.calibration.board_width, config.calibration.board_height


def get_charuco_detector() -> cv2.aruco.CharucoDetector:
    """Initialize the ChArUco board and detector.

    :return: The ChArUco detector.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(config.calibration.aruco_dict)
    detector_params = cv2.aruco.DetectorParameters()
    charuco_params = cv2.aruco.CharucoParameters()

    board = cv2.aruco.CharucoBoard(
        get_board_shape(),
        config.calibration.square_length,
        config.calibration.marker_length,
        dictionary
    )

    return cv2.aruco.CharucoDetector(board, charuco_params, detector_params)


def get_transformed_shape(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[int, int]:
    """Get the transformed shape of the image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :return: The transformed shape of the image.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, shape)

    width = int(max_x - min_x)
    height = int(max_y - min_y)

    return height, width


def find_intersection(
        line1: tuple[Coordinate, Coordinate],
        line2: tuple[Coordinate, Coordinate],
        segments: bool = True
) -> Optional[Coordinate]:
    """Find the intersection between two lines.

    :param line1: The first line.
    :param line2: The second line.
    :param segments: Whether the lines are segments or infinite lines.
    :return: The intersection between the two lines, if it exists.
    """
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

    px = x1 + ua * (x2 - x1)
    py = y1 + ua * (y2 - y1)

    if segments and not (0 <= ua <= 1 and 0 <= ub <= 1):
        return None

    return px, py


def find_offsets(grids: np.ndarray, shapes: np.ndarray, ref_idx: int) -> np.ndarray:
    """Find the offsets for the images.

    :param grids: The grids of the ChArUco boards after warping.
    :param shapes: The shapes of the warped images.
    :param ref_idx: The index of the reference image.
    :return: The offsets for the images.
    """
    if len(grids) != len(shapes):
        raise ValueError("The number of grids and shapes must be the same")

    leftmost_idx = np.argmax([np.max(grid[:, :, 0][np.nonzero(grid[:, :, 0])]) for grid in grids])
    rightmost_idx = np.argmin([np.max(grid[:, :, 0][np.nonzero(grid[:, :, 0])]) for grid in grids])

    offsets = np.zeros((len(grids), 2), dtype=np.int32)
    ref_points = grids[leftmost_idx].reshape(-1, 2)

    for i in range(grids.shape[0]):
        if i == leftmost_idx:
            continue

        for p1, p2 in zip(grids[i].reshape(-1, 2), ref_points):
            if not np.all(p1) or not np.all(p2):
                continue

            offsets[i] = p2 - p1

    # Normalize the offsets
    ref_y = offsets[ref_idx][1]
    offsets[:, 1] -= ref_y

    # Center the reference image
    dist_to_left = offsets[ref_idx][0]
    dist_to_right = offsets[rightmost_idx][0] + shapes[rightmost_idx][1] - (shapes[ref_idx][1] + dist_to_left)

    diff = dist_to_right - dist_to_left
    offsets[:, 0] += diff

    return offsets
