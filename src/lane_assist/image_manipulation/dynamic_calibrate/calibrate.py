import cv2
import numpy as np

from datetime import datetime
from pathlib import Path
from typing import Optional
from common.config import Calibration as Config
from lane_assist.image_manipulation.dynamic_calibrate.birdview import warp_image
from lane_assist.image_manipulation.dynamic_calibrate.utils import corners_to_grid, find_largest_rectangle, \
    euclidean_distance, get_transformed_corners, get_scale_factor


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
    :param shape: The shape of the ChArUco board.
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


def calibrate_cameras(images: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Calibrate multiple cameras using images of a ChArUco board.

    :param images: The images to use for calibration.
    :return: The perspective matrices for each image.
    """
    detector = initialize_charuco_detector()
    src_grids = np.empty((len(images), Config.BOARD_HEIGHT - 1, Config.BOARD_WIDTH - 1, 2), dtype=np.float32)
    all_corners = np.empty((len(images), 4, 2), dtype=np.float32)
    all_shapes = np.empty((len(images), 2), dtype=np.int32)

    min_size = float("inf")
    min_scale_factor = float("inf")

    # Find the corners and shapes of the ChArUco boards.
    for i, image in enumerate(images):
        charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
        if charuco_ids is None or len(charuco_ids) < 4:
            raise ValueError("Not enough ChArUco corners found")

        src_grids[i] = corners_to_grid(charuco_corners, charuco_ids, get_board_shape())
        all_corners[i], all_shapes[i] = find_corners(src_grids[i])
        min_size = min(min_size, get_max_distance(all_corners[i], all_shapes[i]))

    # # Find the minimum scale factor.
    # for image, src_corners, shape in zip(images, all_corners, all_shapes):
    #     dst_corners = remove_perspective(src_corners, shape, min_size)
    #     dst_matrix, _ = cv2.findHomography(src_corners, dst_corners)
    #
    #     h, w = image.shape[:2]
    #     scale_factor = get_scale_factor(dst_matrix, (w, h), Config.MAX_IMAGE_HEIGHT, Config.MAX_IMAGE_WIDTH)
    #     min_scale_factor = min(min_scale_factor, scale_factor)

    dst_corners = remove_perspective(all_corners[1], all_shapes[1], min_size)
    dst_matrix, _ = cv2.findHomography(all_corners[1], dst_corners)

    h, w = images[1].shape[:2]
    min_scale_factor = get_scale_factor(dst_matrix, (h, w), Config.MAX_IMAGE_HEIGHT, Config.MAX_IMAGE_WIDTH)

    # Calculate the perspective matrices for each image.
    dst_matrices = np.empty((len(images), 3, 3), dtype=np.float32)
    for i, (src_corners, shape) in enumerate(zip(all_corners, all_shapes)):
        dst_corners = remove_perspective(src_corners, shape, min_size, min_scale_factor)
        dst_matrices[i] = cv2.findHomography(src_corners, dst_corners)[0]

    # Calculate the destination points for the ChArUco board.
    dst_grids = np.empty((len(images), Config.BOARD_HEIGHT - 1, Config.BOARD_WIDTH - 1, 2), dtype=np.float32)
    for i, (image, src_grid, matrix) in enumerate(zip(images, src_grids, dst_matrices)):
        dst_points = cv2.perspectiveTransform(src_grid.reshape(-1, 1, 2), matrix)

        h, w = image.shape[:2]
        min_x, min_y, _, _ = get_transformed_corners(matrix, (h, w))
        dst_points -= [min_x, min_y]

        zero_mask = np.all(src_grid.reshape(-1, 2) == 0, axis=1)
        dst_points[zero_mask] = [0, 0]

        dst_grids[i] = dst_points.reshape(Config.BOARD_HEIGHT - 1, Config.BOARD_WIDTH - 1, 2)

    return dst_matrices, dst_grids


def find_offsets(images: list[np.ndarray], grids: np.ndarray) -> tuple[np.ndarray, int, int]:
    """Find the offsets for the images.

    :param images: The warped images to find the offsets for.
    :param grids: The grids of the ChArUco boards after warping.
    :return: The offsets for the images.
    """
    if len(images) != 3 or len(grids) != 3:
        raise ValueError("Exactly three images and grids are required")

    offset_left = np.zeros(2)
    offset_center = np.zeros(2)
    offset_right = np.zeros(2)

    center_points = grids[1].reshape(-1, 2)
    for i, p1 in enumerate(grids[0].reshape(-1, 2)):
        if not np.all(p1):
            continue

        p2 = center_points[i]
        if not np.all(p2):
            continue

        offset_left = p2 - p1

    for i, p1 in enumerate(grids[2].reshape(-1, 2)):
        if not np.all(p1):
            continue

        p2 = center_points[i]
        if not np.all(p2):
            continue

        offset_right = p2 - p1

    final_width = max(images[1].shape[1], images[0].shape[1] + offset_left[0], images[2].shape[1] + offset_right[0]) \
        - min(offset_left[0], offset_right[0], 0)
    final_height = max(images[1].shape[0], images[0].shape[0] + offset_left[1], images[2].shape[0] + offset_right[1]) \
        - min(offset_left[1], offset_right[1], 0)

    offsets = np.array([offset_left, offset_center, offset_right], dtype=np.int32)
    return offsets, int(final_width), int(final_height)


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


class CameraCalibrator:
    """A class for calibrating multiple cameras."""

    left_image: Optional[np.ndarray]
    center_image: Optional[np.ndarray]
    right_image: Optional[np.ndarray]

    matrices: np.ndarray
    offsets: np.ndarray
    shape: tuple[int, int]

    _grids: np.ndarray

    def __init__(self, left: np.ndarray = None, center: np.ndarray = None, right: np.ndarray = None) -> None:
        """Initialize the CameraCalibrator.

        :param left: The image from the left camera.
        :param center: The image from the center camera.
        :param right: The image from the right camera.
        """
        self.left_image = left
        self.center_image = center
        self.right_image = right

    def calibrate(self) -> None:
        """Calibrate the cameras."""
        if self.left_image is None or self.center_image is None or self.right_image is None:
            raise ValueError("All calibration images must be set")

        images = [self.left_image, self.center_image, self.right_image]
        matrices, grids = calibrate_cameras(images)

        self.matrices = matrices
        self._grids = grids

    def find_offsets(self) -> None:
        """Find the offsets for the images."""
        if self.left_image is None or self.center_image is None or self.right_image is None:
            raise ValueError("All calibration images must be set")

        if self._grids is None or self.matrices is None:
            raise ValueError("The cameras have not been calibrated")

        warped_center, _ = warp_image(self.center_image, self.matrices[1])
        warped_left, cropped_left = warp_image(self.left_image, self.matrices[0], warped_center.shape[0])
        warped_right, cropped_right = warp_image(self.right_image, self.matrices[2], warped_center.shape[0])

        self._grids[0] = crop_grid(self._grids[0], cropped_left)
        self._grids[2] = crop_grid(self._grids[2], cropped_right)

        images = [warped_left, warped_center, warped_right]
        offsets, width, height = find_offsets(images, self._grids)

        self.offsets = offsets
        self.shape = (height, width)

    def save(self, save_dir: Path | str) -> None:
        """Save the calibration data to a file.

        :param save_dir: The folder to save the calibration data to.
        """
        if self.matrices is None or self.offsets is None:
            raise ValueError("The cameras have not been calibrated")

        save_dir = Path(save_dir)
        history_dir = save_dir / "history"
        history_dir.mkdir(exist_ok=True, parents=True)

        filename = datetime.now().strftime("%m_%d_%Y_%H_%M_%S") + ".npz"
        history_file = history_dir / filename
        latest_file = save_dir / "latest.npz"

        np.savez(history_file, matrices=self.matrices, offsets=self.offsets, shape=self.shape)
        np.savez(latest_file, matrices=self.matrices, offsets=self.offsets, shape=self.shape)

    def load(self, path: Path | str) -> None:
        """Load the calibration data from a file.

        :param path: The path to the calibration data.
        """
        data = np.load(Path(path))

        self.matrices = data["matrices"]
        self.offsets = data["offsets"]
        self.shape = data["shape"]
