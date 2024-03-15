import cv2
import numpy as np

from common.config import Calibration as Config
from lane_assist.image_manipulation.dynamic_calibrate.utils import corners_to_grid, find_largest_rectangle, \
    euclidean_distance, adjust_perspective


def get_board_shape() -> tuple[int, int]:
    """Get the shape of the ChArUco board.

    :return: The shape of the ChArUco board.
    """
    return Config.BOARD_WIDTH, Config.BOARD_HEIGHT


def initialize_charuco_detector() -> cv2.aruco.CharucoDetector:
    """Initialize the ChArUco board and detector.

    :return: The ChArUco detector.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(Config.ARUCO_DICT)
    detector_params = cv2.aruco.DetectorParameters()
    charuco_params = cv2.aruco.CharucoParameters()

    board_shape = get_board_shape()
    board = cv2.aruco.CharucoBoard(board_shape, Config.SQUARE_LENGTH, Config.MARKER_LENGTH, dictionary)

    return cv2.aruco.CharucoDetector(board, charuco_params, detector_params)


def find_corners(image: np.ndarray, detector: cv2.aruco.CharucoDetector) -> tuple[np.ndarray, tuple[int, int]]:
    """Find the ChArUco corners in an image.

    :param image: The image to find the corners in.
    :param detector: The ChArUco detector.
    :return: The corners of the ChArUco board.
    """
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
    if charuco_ids is None or len(charuco_ids) < 4:
        raise ValueError("Not enough ChArUco corners found")

    # Convert the 1D-array of corners and ids to a 2D-array
    grid = corners_to_grid(charuco_corners, charuco_ids, get_board_shape())
    binary_matrix = np.any(grid, axis=2).astype(np.uint8)

    # Find the largest rectangle in the binary matrix
    corner_indices = find_largest_rectangle(binary_matrix)
    if corner_indices is None:
        raise ValueError("No rectangle found in the ChArUco board")

    rect_w = int(euclidean_distance(corner_indices[0], corner_indices[1]))
    rect_h = int(euclidean_distance(corner_indices[1], corner_indices[2]))
    shape = (rect_w, rect_h)

    return grid[corner_indices[:, 0], corner_indices[:, 1]], shape


def get_max_distance(corners: np.ndarray, shape: tuple[int, int]) -> float:
    """Get the maximum distance from one square to another.

    :param corners: The corners to calculate the distance from.
    :param shape: The shape of the original image.
    :return: The maximum distance from one square to another.
    """
    w, h = shape

    dist_t = euclidean_distance(corners[0], corners[1]) / w
    dist_b = euclidean_distance(corners[2], corners[3]) / w
    dist_l = euclidean_distance(corners[0], corners[3]) / h
    dist_r = euclidean_distance(corners[1], corners[2]) / h

    return max(dist_t, dist_b, dist_l, dist_r)


def remove_perspective(
        corners: np.ndarray,
        shape: tuple[int, int],
        length: int,
        scale_factor: float = 1
) -> np.ndarray:
    """Remove perspective from a set of corners.

    :param corners: The corners to remove perspective from.
    :param shape: The shape of the original image.
    :param length: The length of the squares of the ChArUco board.
    :param scale_factor: The scale factor to apply to the corners.
    :return: The corners with perspective removed.
    """
    w, h = shape
    cx, cy = np.mean(corners, axis=0)

    new_w = int(w * length * scale_factor)
    new_h = int(h * length * scale_factor)

    new_corners = np.array([
        [cx - new_h / 2, cy + new_w / 2],
        [cx - new_h / 2, cy - new_w / 2],
        [cx + new_h / 2, cy - new_w / 2],
        [cx + new_h / 2, cy + new_w / 2]
    ])

    if corners[0, 1] < corners[1, 1]:
        new_corners = new_corners[::-1]

    return new_corners


def calibrate_cameras(batch: np.ndarray) -> np.ndarray:
    """Calibrate multiple cameras using images of a ChArUco board.

    :param batch: The images to use for calibration.
    :return: The perspective matrices for each image.
    """
    detector = initialize_charuco_detector()
    all_corners = np.empty((len(batch), 4, 2), dtype=np.float32)
    all_shapes = np.empty((len(batch), 2), dtype=np.int32)

    min_size = float("inf")
    min_scale_factor = float("inf")

    # Find the corners and shapes of the ChArUco boards.
    for i, image in enumerate(batch):
        all_corners[i], all_shapes[i] = find_corners(image, detector)

    # Find the minimum size.
    for src_corners, shape in zip(all_corners, all_shapes):
        max_dist = get_max_distance(src_corners, shape)
        min_size = min(min_size, max_dist)

    # Find the minimum scale factor.
    for i, (src_corners, shape) in enumerate(zip(all_corners, all_shapes)):
        dst_corners = remove_perspective(src_corners, shape, min_size)
        dst_matrix, _ = cv2.findHomography(src_corners, dst_corners)

        _, new_w, new_h = adjust_perspective(dst_matrix, (batch[i].shape[0], batch[i].shape[1]))

        min_scale_factor = min(min_scale_factor, Config.MAX_IMAGE_WIDTH / new_w, Config.MAX_IMAGE_HEIGHT / new_h)

    # Calibrate the cameras.
    dst_matrices = np.empty((len(batch), 3, 3), dtype=np.float32)
    for i, (src_corners, shape) in enumerate(zip(all_corners, all_shapes)):
        dst_corners = remove_perspective(src_corners, shape, min_size, min_scale_factor)
        dst_matrices[i] = cv2.findHomography(src_corners, dst_corners)[0]

    return dst_matrices
