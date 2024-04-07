import cv2
import numpy as np

from config import config
from datetime import datetime
from pathlib import Path
from typing import Optional
from lane_assist.preprocessing.utils.charuco import find_corners
from lane_assist.preprocessing.utils.corners import get_transformed_corners, get_border_of_points
from lane_assist.preprocessing.utils.grid import get_dst_points, corners_to_grid, merge_grids
from lane_assist.preprocessing.utils.other import (
    get_charuco_detector,
    get_transformed_shape,
    find_offsets, euclidean_distance, get_board_shape, find_intersection
)


class CameraCalibrator:
    """A class for calibrating multiple cameras.

    Attributes
    ----------
        board: The ChArUco board.
        camera_matrix: The camera matrix.
        detector: The ChArUco detector.
        dist_coeffs: The distortion coefficients.
        images: The images to calibrate.
        matrices: The perspective matrices.
        offsets: The offsets.
        output_shape: The shape of the output images.
        ref_idx: The index of the reference image.
        shapes: The shapes of the perspective-transformed images.
        topdown_matrix: The top-down matrix.

    """

    board: cv2.aruco.CharucoBoard
    camera_matrix: Optional[np.ndarray]
    detector: cv2.aruco.CharucoDetector
    dist_coeffs: Optional[np.ndarray]
    images: Optional[list[np.ndarray]]
    matrices: Optional[np.ndarray]
    offsets: Optional[np.ndarray]
    output_shape: Optional[tuple[int, int]]
    ref_idx: int
    shapes: Optional[np.ndarray]
    topdown_matrix: Optional[np.ndarray]

    _angle: float = 0.0
    _charuco_corners: list[np.ndarray]
    _charuco_ids: list[np.ndarray]
    _combined_grid: np.ndarray
    _dst_grids: np.ndarray
    _input_shape: tuple[int, int]
    _src_grids: np.ndarray
    _vanishing_line: int = 0

    def __init__(self, images: list[np.ndarray] = None, ref_idx: int = 1, input_shape: tuple[int, int] = None) -> None:
        """Initialize the camera calibrator.

        :param images: The images to calibrate.
        :param ref_idx: The index of the reference image.
        :param input_shape: The shape of the input images.
        """
        self.detector = get_charuco_detector()
        self.board = self.detector.getBoard()

        self.images = images
        self.ref_idx = ref_idx
        self.shapes = np.zeros((len(images), 2), dtype=np.int32)

        self._input_shape = input_shape
        if input_shape is None and images is not None:
            self._input_shape = (images[ref_idx].shape[1], images[ref_idx].shape[0])

        w, h = np.subtract(get_board_shape(), 1)
        self._combined_grid = np.zeros((h, w, 2), dtype=np.float32)
        self._dst_grids = np.zeros((len(images), h, w, 2), dtype=np.float32)
        self._src_grids = np.zeros((len(images), h, w, 2), dtype=np.float32)
        self._charuco_corners = []
        self._charuco_ids = []

    def calibrate(self) -> None:
        """Calibrate the cameras."""
        self.detect_boards()
        self.calibrate_cameras()
        self.calibrate_perspective_matrices()
        self.calibrate_offsets()
        self.calibrate_topdown_matrix()
        self.calibrate_region_of_interest()

    def calibrate_cameras(self) -> None:
        """Calibrate the cameras."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if len(self._charuco_corners) == 0:
            raise ValueError("The ChArUco boards must be detected first")

        all_obj_points = []
        all_img_points = []

        for charuco_corners, charuco_ids in zip(self._charuco_corners, self._charuco_ids):
            obj_points, img_points = self.board.matchImagePoints(charuco_corners, charuco_ids)
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)

        retval, self.camera_matrix, self.dist_coeffs, _, _ = cv2.calibrateCamera(
            all_obj_points, all_img_points, self._input_shape, None, None
        )

    def calibrate_offsets(self) -> None:
        """Calibrate the offsets."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.matrices is None or self._dst_grids is None:
            raise ValueError("The cameras have not been calibrated")

        input_shape = self._input_shape[::-1]
        shapes = np.zeros((len(self.images), 2), dtype=np.int32)
        for i in range(len(self.images)):
            if i == self.ref_idx:
                shapes[i] = input_shape
                continue

            shapes[i] = get_transformed_shape(self.matrices[i], input_shape)

        self.offsets = find_offsets(self._dst_grids, shapes, self.ref_idx) - [0, self._vanishing_line]

    def calibrate_perspective_matrices(self) -> None:
        """Calibrate the matrices."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("The cameras have not been calibrated")

        ref_grid = self._src_grids[self.ref_idx]
        flat_ref = ref_grid.reshape(-1, 2)

        # Calculate the perspective matrices
        self.matrices = np.zeros((len(self.images), 3, 3))
        for i, grid in enumerate(self._src_grids):
            if i == self.ref_idx:
                self.matrices[i] = np.eye(3)
                continue

            flat_dst = grid.reshape(-1, 2)

            ref_points = flat_ref[np.any(flat_ref, axis=1) & np.any(flat_dst, axis=1)]
            dst_points = flat_dst[np.any(flat_ref, axis=1) & np.any(flat_dst, axis=1)]
            if len(dst_points) < 4:
                raise ValueError("The ChArUco boards do not overlap")

            self.matrices[i] = cv2.findHomography(dst_points, ref_points)[0]

        self._calculate_dst_grids()
        self._calculate_vanishing_line()
        self._calculate_angle()

    def calibrate_region_of_interest(self) -> None:
        """Calibrate the region of interest."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.topdown_matrix is None:
            raise ValueError("The top down matrix has not been calibrated")

        w, h = self._input_shape

        # Find the corners of the stitched image
        src_corners = np.zeros((4 * len(self.images), 1, 2), dtype=np.float32)
        for i in range(len(self.images)):
            dst_points = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.float32)
            self.shapes[i] = w, h

            if i != self.ref_idx:
                dst_points = cv2.perspectiveTransform(dst_points, self.matrices[i])
                min_x, min_y, max_x, max_y = get_border_of_points(dst_points[:, 0])

                width = int(max_x - min_x)
                height = int(max_y - min_y)

                self.shapes[i] = width, height

            src_corners[i * 4:i * 4 + 4] = dst_points + self.offsets[i]

        # Find the new location of the corners
        src_corners = np.clip(src_corners, 0, None)
        dst_corners = cv2.perspectiveTransform(src_corners, self.topdown_matrix)
        min_x, min_y, max_x, max_y = get_border_of_points(dst_corners[:, 0])

        # Calculate the new shape of the image
        width = int(max_x - min_x)
        height = int(max_y - min_y)

        scale_factor = min(
            config.calibration.max_image_width / width,
            config.calibration.max_image_height / height
        )

        min_y *= scale_factor
        min_x *= scale_factor
        self.output_shape = int(width * scale_factor), int(height * scale_factor)

        # Adjust the top-down matrix
        adjusted_matrix = np.array([[scale_factor, 0, -min_x], [0, scale_factor, -min_y], [0, 0, 1]])
        self.topdown_matrix = np.dot(adjusted_matrix, self.topdown_matrix)

    def calibrate_topdown_matrix(self) -> None:
        """Calibrate the top-down matrix."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.offsets is None:
            raise ValueError("The offsets have not been calibrated")

        corners, shape = find_corners(self._combined_grid)
        length = euclidean_distance(corners[0], corners[3]) / shape[1]

        flat_grid = self._combined_grid.reshape(-1, 2)

        dst_points = get_dst_points(length, self._angle)
        dst_points = dst_points[np.any(flat_grid, axis=1)]
        src_points = flat_grid[np.any(flat_grid, axis=1)] + self.offsets[self.ref_idx]

        self.topdown_matrix = cv2.findHomography(src_points, dst_points)[0]

    def detect_boards(self) -> None:
        """Detect the ChArUco boards."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        for i, image in enumerate(self.images):
            charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(image)
            if charuco_corners is None or len(charuco_corners) < 4:
                raise ValueError("The ChArUco board was not detected")

            charuco_corners *= self._get_scale(image)

            self._charuco_corners.append(charuco_corners)
            self._charuco_ids.append(charuco_ids)
            self._src_grids[i] = corners_to_grid(charuco_corners, charuco_ids, get_board_shape())

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
            output_shape=self.output_shape,
            shapes=self.shapes,
            topdown_matrix=self.topdown_matrix
        )

        np.savez(history_file, **arrays)
        np.savez(latest_file, **arrays)

    def _calculate_angle(self) -> None:
        """Calculate the angle of the board."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("The cameras have not been calibrated")

        obj_points, img_points = self.board.matchImagePoints(
            self._charuco_corners[self.ref_idx],
            self._charuco_ids[self.ref_idx]
        )

        retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
        rmat, _ = cv2.Rodrigues(rvec)

        angles = cv2.RQDecomp3x3(rmat)[0]
        self._angle = np.radians(angles[2])

    def _calculate_dst_grids(self) -> None:
        """Calculate the destination grids."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.matrices is None:
            raise ValueError("The cameras have not been calibrated")

        for i, (image, matrix, grid) in enumerate(zip(self.images, self.matrices, self._src_grids)):
            if i == self.ref_idx:
                merge_grids(self._combined_grid, grid)
                self._dst_grids[i] = grid
                continue

            min_x, min_y = get_transformed_corners(matrix, image.shape[:2])[:2]
            dst_grid = cv2.perspectiveTransform(grid.reshape(-1, 1, 2), matrix).reshape(grid.shape)
            dst_grid[np.all(grid == 0, axis=2)] = 0

            merge_grids(self._combined_grid, dst_grid)

            dst_grid[np.any(grid, axis=2)] -= [min_x, min_y]
            self._dst_grids[i] = dst_grid

            adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
            self.matrices[i] = np.dot(adjusted_matrix, matrix)

    def _calculate_vanishing_line(self) -> None:
        """Calculate the vanishing line."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self._combined_grid is None:
            raise ValueError("The cameras have not been calibrated")

        lines = [[point for point in row if np.any(point)] for row in self._combined_grid]
        lines = [(line[0], line[-1]) for line in lines if len(line) > 1]

        intersections = [find_intersection(lines[i], lines[j])
                         for i in range(len(lines) - 1)
                         for j in range(i + 1, len(lines))]
        intersections = [point for point in intersections if point is not None]
        intersections = np.array(intersections)

        if len(intersections) > 0:
            vanishing_line = np.median(intersections, axis=0)[1]
            vanishing_line *= 1 + config.calibration.vanishing_line_offset

            self._vanishing_line = int(vanishing_line)

    def _get_scale(self, image: np.ndarray) -> float:
        """Get the scale factor for the image.

        :param image: The image to get the scale factor for.
        :return: The scale factor for the image.
        """
        return max(self._input_shape[1] / image.shape[0], self._input_shape[0] / image.shape[1])

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
        calibrator.output_shape = tuple(data["output_shape"])
        calibrator.shapes = data["shapes"]
        calibrator.topdown_matrix = data["topdown_matrix"]

        return calibrator
