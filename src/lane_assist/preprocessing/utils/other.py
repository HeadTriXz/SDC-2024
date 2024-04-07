import cv2
import numpy as np

from config import config
from lane_assist.preprocessing.utils.corners import get_transformed_corners
from typing import Optional

Coordinate = tuple[int, int] | np.ndarray


def calculate_stitched_shape(offsets: np.ndarray, shapes: np.ndarray) -> tuple[int, int]:
    """Calculate the output shape for the stitched image.

    :param offsets: The offsets for the images.
    :param shapes: The shapes of the images.
    :return: The output shape for the stitched image (width, height).
    """
    width_max = max(shape[0] + offset[0] for shape, offset in zip(shapes, offsets))
    width_min = min(0, min(offset[0] for offset in offsets))
    width = int(width_max - width_min)

    height = max(shape[1] + offset[1] for shape, offset in zip(shapes, offsets))

    return width, height


def euclidean_distance(p1: np.ndarray | Coordinate, p2: np.ndarray | Coordinate) -> float:
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
        line2: tuple[Coordinate, Coordinate]
) -> Optional[Coordinate]:
    """Find the intersection between two lines.

    :param line1: The first line.
    :param line2: The second line.
    :return: The intersection between the two lines, if it exists.
    """
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a: tuple[int, int], b: tuple[int, int]) -> int:
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) // div
    y = det(d, ydiff) // div

    return x, y


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
    offsets = np.zeros((len(grids), 2), dtype=np.int32)
    ref_points = grids[leftmost_idx].reshape(-1, 2)

    for i in range(grids.shape[0]):
        if i == leftmost_idx:
            continue

        for p1, p2 in zip(grids[i].reshape(-1, 2), ref_points):
            if not np.all(p1) or not np.all(p2):
                continue

            offsets[i] = p2 - p1

    ref_y = offsets[ref_idx][1]
    offsets[:, 1] -= ref_y

    return offsets
