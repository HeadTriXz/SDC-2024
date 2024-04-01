import cv2
import numpy as np

from config import config
from datetime import datetime
from pathlib import Path
from typing import Optional
from lane_assist.preprocessing.utils.charuco import find_corners
from lane_assist.preprocessing.utils.corners import get_dst_corners, get_transformed_corners
from lane_assist.preprocessing.utils.grid import get_dst_grid, crop_grid, corners_to_grid
from lane_assist.preprocessing.utils.other import (
    get_charuco_detector,
    get_scale_factor,
    get_transformed_shape,
    find_offsets, euclidean_distance, get_board_shape
)


class CameraCalibrator:
    """A class for calibrating multiple cameras.

    Attributes
    ----------
        images: The images to calibrate.
        ref_idx: The index of the reference image.
        camera_matrix: The camera matrix.
        dist_coeffs: The distortion coefficients.
        matrices: The perspective matrices.
        offsets: The offsets.
        output_shape: The shape of the output images.

    """

    images: Optional[list[np.ndarray]]
    ref_idx: int

    camera_matrix: Optional[np.ndarray]
    dist_coeffs: Optional[np.ndarray]
    matrices: Optional[np.ndarray]
    offsets: Optional[np.ndarray]
    output_shape: tuple[int, int]

    _angle: float = 0.0
    _grids: np.ndarray
    _input_shape: tuple[int, int]

    def __init__(self, images: list[np.ndarray] = None, ref_idx: int = 1, input_shape: tuple[int, int] = None) -> None:
        """Initialize the camera calibrator.

        :param images: The images to calibrate.
        :param ref_idx: The index of the reference image.
        :param input_shape: The shape of the input images.
        """
        self.images = images
        self.ref_idx = ref_idx

        self._input_shape = input_shape
        if input_shape is None and images is not None:
            self._input_shape = (images[ref_idx].shape[1], images[ref_idx].shape[0])

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

            scale = self._get_scale(image)
            charuco_corners *= scale

            obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)

        ref_shape = self._input_shape[::-1]
        retval, self.camera_matrix, self.dist_coeffs, _, _ = cv2.calibrateCamera(
            all_obj_points, all_img_points, ref_shape, None, None
        )

    def calibrate_matrices(self) -> None:
        """Calibrate the matrices."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("The cameras have not been calibrated")

        detector = get_charuco_detector()
        src_grids = self._get_src_grids(detector)
        all_src_corners, all_shapes = zip(*[find_corners(grid) for grid in src_grids])

        # Calculate the scale factor
        ref_src_corners = all_src_corners[self.ref_idx]
        max_dist = euclidean_distance(ref_src_corners[0], ref_src_corners[3]) / all_shapes[self.ref_idx][1]

        ref_dst_grid = get_dst_grid(max_dist, self._angle)
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
            dst_corners = get_dst_corners(max_dist, self._angle, shape, scale_factor)
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
        self.output_shape = (height, width)

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
            shape=self.output_shape
        )

        np.savez(history_file, **arrays)
        np.savez(latest_file, **arrays)

    def _get_scale(self, image: np.ndarray) -> float:
        """Get the scale factor for the image.

        :param image: The image to get the scale factor for.
        :return: The scale factor for the image.
        """
        return max(self._input_shape[1] / image.shape[0], self._input_shape[0] / image.shape[1])

    def _get_src_grids(self, detector: cv2.aruco.CharucoDetector) -> list[np.ndarray]:
        """Get the source grids for the images.

        :param detector: The ChArUco detector.
        :return: The source grids for the images.
        """
        board = detector.getBoard()
        grids = []

        for i, image in enumerate(self.images):
            scale = self._get_scale(image)

            charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
            if charuco_corners is None or len(charuco_corners) < 4:
                raise ValueError("The ChArUco board was not detected")

            charuco_corners *= scale
            grid = corners_to_grid(charuco_corners, charuco_ids, get_board_shape())
            grids.append(grid)

            self.images[i] = cv2.resize(image, self._input_shape)
            if i == self.ref_idx:
                obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
                retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)

                rmat, _ = cv2.Rodrigues(rvec)
                pmat = np.dot(self.camera_matrix, np.hstack((rmat, tvec)))

                rz = cv2.decomposeProjectionMatrix(pmat)[-1][2][0]
                self._angle = np.radians(rz)

        return grids

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
        calibrator.output_shape = data["shape"]

        return calibrator
