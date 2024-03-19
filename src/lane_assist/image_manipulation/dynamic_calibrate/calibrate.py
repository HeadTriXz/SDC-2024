import cv2
import numpy as np

from datetime import datetime
from pathlib import Path
from typing import Optional
from common.config import Calibration as Config
from lane_assist.image_manipulation.dynamic_calibrate.utils import get_scale_factor, get_charuco_detector, \
    find_corners, get_slope, get_transformed_shape, find_offsets, get_src_grid, get_dst_grid


class CameraCalibrator:
    """A class for calibrating multiple cameras."""

    image_left: Optional[np.ndarray]
    image_center: Optional[np.ndarray]
    image_right: Optional[np.ndarray]

    camera_matrix: Optional[np.ndarray]
    dist_coeffs: Optional[np.ndarray]
    matrices: Optional[np.ndarray]
    offsets: Optional[np.ndarray]
    shape: tuple[int, int]

    _grids: np.ndarray

    def __init__(self, left: np.ndarray = None, center: np.ndarray = None, right: np.ndarray = None) -> None:
        """Initialize the camera calibrator.

        :param left: The image from the left camera.
        :param center: The image from the center camera.
        :param right: The image from the right camera.
        """
        self.image_left = left
        self.image_center = center
        self.image_right = right

    @property
    def images(self) -> list[np.ndarray]:
        """Return the images."""
        if self.image_left is None or self.image_center is None or self.image_right is None:
            raise ValueError("The images have not been set")

        return [self.image_left, self.image_center, self.image_right]

    def calibrate(self) -> None:
        """Calibrate the cameras."""
        self.calibrate_cameras()
        self.calibrate_matrices()
        self.calibrate_offsets()

    def calibrate_cameras(self) -> None:
        """Calibrate the cameras."""
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

        retval, self.camera_matrix, self.dist_coeffs, _, _ = cv2.calibrateCamera(all_obj_points, all_img_points, self.image_center.shape, None, None)

    def calibrate_matrices(self) -> None:
        """Calibrate the matrices."""
        detector = get_charuco_detector()
        src_grids = [get_src_grid(detector, image) for image in self.images]
        all_src_corners, all_shapes = zip(*[find_corners(grid) for grid in src_grids])

        h_change, v_change = get_slope(all_src_corners[1][0], all_src_corners[1][3], all_shapes[1][1])
        base_dst_grid = get_dst_grid(h_change, v_change)

        # Calculate the scale factor
        center_dst_grid = base_dst_grid.copy()
        center_dst_grid[src_grids[1][:, :, 0] == 0] = 0

        center_dst_corners, _ = find_corners(center_dst_grid)
        center_matrix, _ = cv2.findHomography(all_src_corners[1], center_dst_corners)

        h, w = self.image_center.shape[:2]
        scale_factor = get_scale_factor(center_matrix, (h, w), Config.MAX_IMAGE_HEIGHT, Config.MAX_IMAGE_WIDTH)
        h_change *= scale_factor
        v_change *= scale_factor

        # Scale the destination grids
        base_dst_grid = get_dst_grid(h_change, v_change)
        dst_grids = np.zeros((len(src_grids), *base_dst_grid.shape), dtype=np.float32)
        for i, src_grid in enumerate(src_grids):
            dst_grids[i] = base_dst_grid.copy()
            dst_grids[i, np.all(src_grid == 0, axis=2)] = 0

        all_dst_corners, _ = zip(*[find_corners(grid) for grid in dst_grids])

        # START DEBUGGING: Draw the grids
        for image, src_grid, dst_grid, src_corners, dst_corners in zip(self.images, src_grids, dst_grids, all_src_corners, all_dst_corners):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for i, point in enumerate(dst_grid.reshape(-1, 2).astype(int)):
                if point[0] == 0 and point[1] == 0:
                    continue

                cv2.circle(image, tuple(point), 5, (0, 255, 0), -1)
                cv2.putText(image, str(i), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            for i, point in enumerate(src_grid.reshape(-1, 2).astype(int)):
                if point[0] == 0 and point[1] == 0:
                    continue

                cv2.circle(image, tuple(point), 5, (0, 0, 255), -1)
                cv2.putText(image, str(i), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            for i, point in enumerate(src_corners.astype(int)):
                cv2.circle(image, tuple(point), 5, (255, 0, 0), -1)
                cv2.putText(image, str(i), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            for i, point in enumerate(dst_corners.astype(int)):
                cv2.circle(image, tuple(point), 5, (255, 0, 0), -1)
                cv2.putText(image, str(i), tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # END DEBUGGING

        self._grids = dst_grids
        self.matrices = np.zeros((len(self.images), 3, 3), dtype=np.float32)
        for i, (src_corners, dst_corners) in enumerate(zip(all_src_corners, all_dst_corners)):
            self.matrices[i] = cv2.findHomography(src_corners, dst_corners)[0]

    def calibrate_offsets(self) -> None:
        """Calibrate the offsets."""
        if self.matrices is None or self._grids is None:
            raise ValueError("The cameras have not been calibrated")

        shapes = np.array([get_transformed_shape(matrix, image.shape) for matrix, image in zip(self.matrices, self.images)])
        self.offsets, width, height = find_offsets(self._grids, shapes)
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
