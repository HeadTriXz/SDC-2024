import cv2
import numpy as np

from src.calibration.utils.corners import get_transformed_corners
from src.config import config


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
