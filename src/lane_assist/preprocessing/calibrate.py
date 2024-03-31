import cv2
import numpy as np

from config import config
from datetime import datetime
from pathlib import Path
from typing import Optional
from lane_assist.preprocessing.utils.charuco import find_corners
from lane_assist.preprocessing.utils.corners import get_dst_corners, get_transformed_corners
from lane_assist.preprocessing.utils.grid import get_src_grid, get_dst_grid, crop_grid
from lane_assist.preprocessing.utils.other import (
    get_charuco_detector,
    get_slope,
    get_scale_factor,
    get_transformed_shape,
    find_offsets
)


class CameraCalibrator:
    """A class for calibrating multiple cameras."""

    images: Optional[list[np.ndarray]]
    ref_idx: int

    camera_matrix: Optional[np.ndarray]
    dist_coeffs: Optional[np.ndarray]
    matrices: Optional[np.ndarray]
    offsets: Optional[np.ndarray]
    shape: tuple[int, int]

    _grids: np.ndarray

    def __init__(self, images: list[np.ndarray] = None, ref_idx: int = 1) -> None:
        """Initialize the camera calibrator.

        :param images: The images to calibrate.
        :param ref_idx: The index of the reference image.
        """
        self.images = images
        self.ref_idx = ref_idx

    def calibrate(self) -> None:
        """Calibrate the cameras."""
        self.calibrate_cameras()
        self.calibrate_matrices()
        self.calibrate_offsets()

    def calibrate_cameras(self) -> None:
        """Calibrate the cameras."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        detector = get_charuco_detector()
        board = detector.getBoard()

        all_obj_points = []
        all_img_points = []

        for image in self.images:
            charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
            if charuco_corners is None or len(charuco_corners) < 4:
                raise ValueError("The ChArUco board was not detected")

            obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)

        ref_shape = self.images[self.ref_idx].shape[:2]
        retval, self.camera_matrix, self.dist_coeffs, _, _ = cv2.calibrateCamera(
            all_obj_points, all_img_points, ref_shape, None, None
        )

    def calibrate_matrices(self) -> None:
        """Calibrate the matrices."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        detector = get_charuco_detector()
        src_grids = [get_src_grid(detector, image) for image in self.images]
        all_src_corners, all_shapes = zip(*[find_corners(grid) for grid in src_grids])

        # Calculate the scale factor
        ref_src_corners = all_src_corners[self.ref_idx]
        h_change, v_change = get_slope(ref_src_corners[0], ref_src_corners[3], all_shapes[self.ref_idx][1])

        ref_dst_grid = get_dst_grid(h_change, v_change)
        ref_dst_grid[np.all(src_grids[self.ref_idx] == 0, axis=2)] = 0

        ref_dst_corners, _ = find_corners(ref_dst_grid)
        ref_matrix, _ = cv2.findHomography(ref_src_corners, ref_dst_corners)

        h, w = self.images[self.ref_idx].shape[:2]
        scale_factor = get_scale_factor(
            ref_matrix,
            (h, w),
            config.calibration.max_image_height,
            config.calibration.max_image_width
        )

        # Calculate the perspective matrices.
        self.matrices = np.zeros((len(self.images), 3, 3), dtype=np.float32)
        for i, (src_corners, shape) in enumerate(zip(all_src_corners, all_shapes)):
            dst_corners = get_dst_corners(src_corners, h_change, v_change, shape, scale_factor)
            self.matrices[i] = cv2.findHomography(src_corners, dst_corners)[0]

        # Calculate the destination points of the ChArUco board.
        self._grids = np.zeros((len(self.images), *ref_dst_grid.shape), dtype=np.float32)
        for i, (image, matrix, src_grid) in enumerate(zip(self.images, self.matrices, src_grids)):
            dst_grid = cv2.perspectiveTransform(src_grid.reshape(-1, 1, 2), matrix).reshape(src_grid.shape)

            h, w = image.shape[:2]
            min_x, min_y = get_transformed_corners(matrix, (h, w))[:2]
            dst_grid -= [min_x, min_y]

            dst_grid[np.all(src_grid == 0, axis=2)] = 0
            self._grids[i] = dst_grid

    def calibrate_offsets(self) -> None:
        """Calibrate the offsets."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.matrices is None or self._grids is None:
            raise ValueError("The cameras have not been calibrated")

        h, w = self.images[self.ref_idx].shape[:2]
        ref_shape, _ = get_transformed_shape(self.matrices[self.ref_idx], (h, w))

        shapes = np.zeros((len(self.images), 2), dtype=np.int32)
        for i, (matrix, image) in enumerate(zip(self.matrices, self.images)):
            if i == self.ref_idx:
                shapes[i] = ref_shape
                continue

            shapes[i], cropped = get_transformed_shape(matrix, image.shape[:2], ref_shape[0])
            self._grids[i] = crop_grid(self._grids[i], cropped)

        self.offsets, width, height = find_offsets(self._grids, shapes, self.ref_idx)
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

        arrays = dict(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            matrices=self.matrices,
            offsets=self.offsets,
            shape=self.shape
        )

        np.savez(history_file, **arrays)
        np.savez(latest_file, **arrays)

    @staticmethod
    def load(path: Path | str) -> "CameraCalibrator":
        """Load the calibration data from a file.

        :param path: The path to the calibration data.
        :return: The camera calibrator.
        """
        data = np.load(Path(path))

        calibrator = CameraCalibrator()
        calibrator.camera_matrix = data["camera_matrix"]
        calibrator.dist_coeffs = data["dist_coeffs"]
        calibrator.matrices = data["matrices"]
        calibrator.offsets = data["offsets"]
        calibrator.shape = data["shape"]

        return calibrator
