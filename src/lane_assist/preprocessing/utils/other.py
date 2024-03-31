import cv2
import numpy as np

from config import config
from lane_assist.preprocessing.utils.corners import get_transformed_corners

Coordinate = tuple[int, int] | np.ndarray


def euclidean_distance(p1: np.ndarray | Coordinate, p2: np.ndarray | Coordinate) -> float:
    """Calculate the Euclidean distance between two points.

    :param p1: The first point.
    :param p2: The second point.
    :return: The Euclidean distance between the two points.
    """
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def get_slope(p1: Coordinate, p2: Coordinate, width: int) -> np.ndarray:
    """Get the slope of a line.

    :param p1: The first point.
    :param p2: The second point.
    :param width: The width of the image.
    :return: The horizontal and vertical change of the line.
    """
    return np.array([p2[0] - p1[0], p2[1] - p1[1]]) / width


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


def get_transformed_shape(
        matrix: np.ndarray,
        shape: tuple[int, int],
        max_height: int = None
) -> tuple[tuple[int, int], int]:
    """Get the transformed shape of the image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :param max_height: The maximum height of the new image.
    :return: The transformed shape of the image.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, shape)

    cropped = 0
    width = int(max_x - min_x)
    height = int(max_y - min_y)
    if max_height is not None and height > max_height:
        cropped = height - max_height
        height = max_height

    return (height, width), cropped


def get_scale_factor(matrix: np.ndarray, shape: tuple[int, int], max_height: int, max_width: int) -> float:
    """Get the scale factor for the perspective matrix.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :param max_height: The maximum height of the new image.
    :param max_width: The maximum width of the new image.
    :return: The scale factor for the perspective matrix.
    """
    new_h, new_w = get_transformed_shape(matrix, shape)[0]
    return min(max_width / new_w, max_height / new_h)


def find_offsets(grids: np.ndarray, shapes: np.ndarray, ref_idx: int = 1) -> tuple[np.ndarray, int, int]:
    """Find the offsets for the images.

    :param grids: The grids of the ChArUco boards after warping.
    :param shapes: The shapes of the warped images.
    :param ref_idx: The index of the reference image (defaults to the second image).
    :return: The offsets for the images.
    """
    if len(grids) != len(shapes):
        raise ValueError("The number of grids and shapes must be the same")

    if ref_idx < 0 or ref_idx >= len(grids):
        raise ValueError("The reference index must be between 0 and the number of grids")

    offsets = np.zeros((len(grids), 2), dtype=np.int32)
    ref_points = grids[ref_idx].reshape(-1, 2)

    for i in range(grids.shape[0]):
        if i == ref_idx:
            continue

        for p1, p2 in zip(grids[i].reshape(-1, 2), ref_points):
            if not np.all(p1) or not np.all(p2):
                continue

            offsets[i] = p2 - p1

    width_max = max(shape[1] + offset[0] for shape, offset in zip(shapes, offsets))
    width_min = min(0, min(offset[0] for offset in offsets))
    width = int(width_max - width_min)

    height_max = max(shape[0] + offset[1] for shape, offset in zip(shapes, offsets))
    height_min = min(0, min(offset[1] for offset in offsets))
    height = int(height_max - height_min)

    return offsets, width, height
