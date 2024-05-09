import cv2
import numpy as np

from datetime import datetime
from pathlib import Path
from typing import Optional

from src.calibration.utils.charuco import find_corners
from src.calibration.utils.corners import get_border_of_points, get_transformed_corners
from src.calibration.utils.grid import corners_to_grid, get_dst_points, merge_grids
from src.calibration.utils.other import (
    euclidean_distance,
    find_intersection,
    find_offsets,
    get_board_shape,
    get_charuco_detector,
    get_transformed_shape,
)
from src.config import config


class CameraCalibrator:
    """A class for calibrating multiple cameras.

    Attributes
    ----------
        board: The ChArUco board.
        detector: The ChArUco detector.
        images: The images to calibrate.
        matrices: The perspective matrices.
        offsets: The offsets.
        output_shape: The shape of the output images.
        ref_idx: The index of the reference image.
        shapes: The shapes of the perspective-transformed images.
        stitched_shape: The shape of the stitched image.
        topdown_matrix: The top-down matrix.

    """

    board: cv2.aruco.CharucoBoard
    detector: cv2.aruco.CharucoDetector
    images: Optional[list[np.ndarray]]
    matrices: Optional[np.ndarray]
    offsets: Optional[np.ndarray]
    output_shape: Optional[tuple[int, int]]
    pixels_per_meter: Optional[float]
    ref_idx: int
    shapes: Optional[np.ndarray]
    stitched_shape: Optional[tuple[int, int]]
    topdown_matrix: Optional[np.ndarray]

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

        self._input_shape = input_shape
        if input_shape is None and images is not None:
            self._input_shape = (images[ref_idx].shape[1], images[ref_idx].shape[0])

        self._charuco_corners = []
        self._charuco_ids = []

    def calibrate(self) -> None:
        """Calibrate the cameras."""
        self.detect_boards()
        self.calibrate_perspective_matrices()
        self.calibrate_offsets()
        self.calibrate_topdown_matrix()
        self.calibrate_angle()
        self.calibrate_region_of_interest()

    def calibrate_offsets(self) -> None:
        """Calculate the offsets of the images in the stitched image."""
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
        self.stitched_shape = self._calculate_stitched_shape()

    def calibrate_perspective_matrices(self) -> None:
        """Calculate the matrices for stitching the images."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self._src_grids is None:
            raise ValueError("The ChArUco boards have not been detected")

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

    def calibrate_angle(self) -> None:
        """Calibrate the angle of the top-down matrix."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.topdown_matrix is None:
            raise ValueError("The top-down matrix has not been calibrated")

        w, h = self._input_shape

        # Find the corners of the stitched image
        src_corners = np.zeros((4 * len(self.images), 1, 2), dtype=np.float32)
        for i in range(len(self.images)):
            dst_points = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.float32)
            if i != self.ref_idx:
                dst_points = cv2.perspectiveTransform(dst_points, self.matrices[i])

            src_corners[i * 4 : i * 4 + 4] = dst_points + self.offsets[i]

        # Find the new location of the corners
        dst_corners = cv2.perspectiveTransform(src_corners, self.topdown_matrix)

        src_corners = src_corners.reshape(-1, 2)
        dst_corners = dst_corners.reshape(-1, 2)

        # Find the angle of the top-down matrix
        src_centroid = np.mean(src_corners, axis=0)
        dst_centroid = np.mean(dst_corners, axis=0)

        src_centered = src_corners - src_centroid
        dst_centered = dst_corners - dst_centroid

        cov_matrix = np.dot(src_centered.T, dst_centered)
        u, s, vh = np.linalg.svd(cov_matrix)

        rmat = np.dot(vh.T, u.T)
        angle = -np.arctan2(rmat[1, 0], rmat[0, 0])

        # Adjust the top-down matrix
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])

        self.topdown_matrix = np.dot(rotation_matrix, self.topdown_matrix)

    def calibrate_region_of_interest(self) -> None:
        """Calibrate the region of interest."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.topdown_matrix is None:
            raise ValueError("The top-down matrix has not been calibrated")

        self.shapes = np.zeros((len(self.images), 2), dtype=np.int32)
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

            src_corners[i * 4 : i * 4 + 4] = dst_points + self.offsets[i]

        # Find the new location of the corners
        src_corners = np.clip(src_corners, 0, None)
        dst_corners = cv2.perspectiveTransform(src_corners, self.topdown_matrix)
        min_x, min_y, max_x, max_y = get_border_of_points(dst_corners[:, 0])

        # Find the new center of the image
        src_center = np.array([[[w // 2, h]]], dtype=np.float32) + self.offsets[self.ref_idx]
        dst_center = cv2.perspectiveTransform(src_center, self.topdown_matrix)[0][0]

        # Update the render distance
        render_front = config.calibration.render_distance.front * self.pixels_per_meter
        render_side = config.calibration.render_distance.side * self.pixels_per_meter

        min_x = max(min_x, dst_center[0] - render_side)
        max_x = min(max_x, dst_center[0] + render_side)
        min_y = max(min_y, dst_center[1] - render_front)
        max_y = dst_center[1]

        # Adjust the left and right sides of the image
        dist_to_left = dst_center[0] - min_x
        dist_to_right = max_x - dst_center[0]
        diff = dist_to_left - dist_to_right

        min_x = max(min_x, min_x + diff)
        max_x = min(max_x, max_x + diff)

        # Find the new top of the image
        lines = [dst_corners[i * 4 : i * 4 + 2, 0] for i in range(len(self.images))]
        min_x_line = np.array([(min_x, min_y), (min_x, max_y)])
        max_x_line = np.array([(max_x, min_y), (max_x, max_y)])

        intersections = [find_intersection(lines[i], min_x_line) for i in range(len(lines))]
        intersections += [find_intersection(lines[i], max_x_line) for i in range(len(lines))]
        intersections = [point for point in intersections if point is not None]

        if len(intersections) > 0:
            min_y = min([point[1] for point in intersections])

        # Calculate the new shape of the image
        width = int(max_x - min_x)
        height = int(max_y - min_y)

        scale_factor = min(config.calibration.max_image_width / width, config.calibration.max_image_height / height)

        min_x *= scale_factor
        min_y *= scale_factor
        self.pixels_per_meter *= scale_factor
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

        # Calculate the pixels per meter
        self.pixels_per_meter = length / self.board.getSquareLength()

        # Calculate the source and destination points
        flat_grid = self._combined_grid.reshape(-1, 2)

        dst_points = get_dst_points(length)
        dst_points = dst_points[np.any(flat_grid, axis=1)]
        src_points = flat_grid[np.any(flat_grid, axis=1)] + self.offsets[self.ref_idx]

        self.topdown_matrix = cv2.findHomography(src_points, dst_points)[0]

    def detect_boards(self) -> None:
        """Detect the ChArUco boards."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        w, h = np.subtract(get_board_shape(), 1)
        self._src_grids = np.zeros((len(self.images), h, w, 2), dtype=np.float32)

        for i, image in enumerate(self.images):
            charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(image)
            if charuco_corners is None or len(charuco_corners) < 4:
                raise ValueError("The ChArUco board was not detected")

            charuco_corners *= self._get_scale(image)

            self._charuco_corners.append(charuco_corners)
            self._charuco_ids.append(charuco_ids)
            self._src_grids[i] = corners_to_grid(charuco_corners, charuco_ids, get_board_shape())

    def save(self, save_dir: Path | str) -> Path:
        """Save the calibration data to a file.

        :param save_dir: The folder to save the calibration data to.
        :return: The path to the saved file.
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
            input_shape=self._input_shape,
            matrices=self.matrices,
            offsets=self.offsets,
            output_shape=self.output_shape,
            pixels_per_meter=self.pixels_per_meter,
            ref_idx=self.ref_idx,
            shapes=self.shapes,
            stitched_shape=self.stitched_shape,
            topdown_matrix=self.topdown_matrix,
        )

        np.savez(history_file, **arrays)
        np.savez(latest_file, **arrays)
        return history_file

    def _calculate_dst_grids(self) -> None:
        """Calculate the destination grids."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.matrices is None:
            raise ValueError("The cameras have not been calibrated")

        self._combined_grid = np.zeros(self._src_grids.shape[1:], dtype=np.float32)
        self._dst_grids = np.zeros_like(self._src_grids, dtype=np.float32)

        for i, (matrix, grid) in enumerate(zip(self.matrices, self._src_grids)):
            if i == self.ref_idx:
                merge_grids(self._combined_grid, grid)
                self._dst_grids[i] = grid
                continue

            min_x, min_y = get_transformed_corners(matrix, self._input_shape[::-1])[:2]
            dst_grid = cv2.perspectiveTransform(grid.reshape(-1, 1, 2), matrix).reshape(grid.shape)
            dst_grid[np.all(grid == 0, axis=2)] = 0

            merge_grids(self._combined_grid, dst_grid)

            dst_grid[np.any(grid, axis=2)] -= [min_x, min_y]
            self._dst_grids[i] = dst_grid

            adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
            self.matrices[i] = np.dot(adjusted_matrix, matrix)

    def _calculate_intersections(self, vertical: bool = False) -> float:
        """Calculate the intersections of the ChArUco board.

        :param vertical: Whether to use the vertical lines.
        :return: The intersections of the ChArUco board.
        """
        # Get the grid of the ChArUco board
        grid = self._combined_grid
        if vertical:
            grid = grid.transpose(1, 0, 2)

        # Find the lines of the ChArUco board
        lines = [[point for point in row if np.any(point)] for row in grid]
        lines = [(line[0], line[-1]) for line in lines if len(line) > 1]

        # Find the intersections of the lines
        intersections = [
            find_intersection(lines[i], lines[j], False)
            for i in range(len(lines) - 1)
            for j in range(i + 1, len(lines))
        ]
        intersections = [point for point in intersections if point is not None]
        intersections = np.array(intersections)

        if len(intersections) == 0:
            return np.nan

        vanishing_line = np.median(intersections, axis=0)[1]
        vanishing_line *= 1 + config.calibration.vanishing_line_offset

        return vanishing_line

    def _calculate_stitched_shape(self) -> tuple[int, int]:
        """Calculates the shape of the stitched image.

        :return: The shape of the stitched image.
        """
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self.offsets is None:
            raise ValueError("The offsets have not been calibrated")

        height = self._input_shape[1] - self._vanishing_line

        # Calculate the corners of each part of the stitched image.
        corners = np.zeros((len(self.images), 4, 2), dtype=np.float32)
        for i in range(len(self.images)):
            if i == self.ref_idx:
                offset = self.offsets[i]
                corners[i] = np.array(
                    [
                        [offset[0], offset[1]],
                        [offset[0] + self._input_shape[0], offset[1]],
                        [offset[0] + self._input_shape[0], offset[1] + height],
                        [offset[0], offset[1] + height],
                    ]
                )
                continue

            h, w = self._input_shape[::-1]
            src_points = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.float32)
            dst_points = cv2.perspectiveTransform(src_points, self.matrices[i])

            corners[i] = dst_points[:, 0] + self.offsets[i]

        # Get the leftmost and rightmost corners
        leftmost_idx = np.argmin([np.min(corners[i][:, 0]) for i in range(len(corners))])
        rightmost_idx = np.argmax([np.max(corners[i][:, 0]) for i in range(len(corners))])

        leftmost = corners[leftmost_idx]
        rightmost = corners[rightmost_idx]

        left_line = (leftmost[0], leftmost[1])
        right_line = (rightmost[0], rightmost[1])

        # Find the point where the top edge intersects with the bottom of the stitched image.
        min_x = 0
        max_x = rightmost[1][0]

        y_line = np.array([(min_x, height), (max_x, height)], dtype=np.float32)

        left_intersection = find_intersection(left_line, y_line)
        right_intersection = find_intersection(right_line, y_line)

        # Calculate the new width of the image.
        if left_intersection is not None:
            min_x = int(left_intersection[0])
            self.offsets -= [min_x, 0]

        if right_intersection is not None:
            max_x = int(right_intersection[0])

        return max_x - min_x, height

    def _calculate_vanishing_line(self) -> None:
        """Calculate the vanishing line."""
        if self.images is None:
            raise ValueError("No images to calibrate")

        if self._combined_grid is None:
            raise ValueError("The cameras have not been calibrated")

        h_line = self._calculate_intersections(False)
        v_line = self._calculate_intersections(True)

        vanishing_line = min(h_line, v_line)
        if vanishing_line > 0:
            self._vanishing_line = int(vanishing_line)

    def _get_scale(self, image: np.ndarray) -> float:
        """Get the scale factor for the image.

        :param image: The image to get the scale factor for.
        :return: The scale factor for the image.
        """
        return max(self._input_shape[1] / image.shape[0], self._input_shape[0] / image.shape[1])
