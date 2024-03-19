import cv2
import numpy as np

from common.config import Calibration as Config
from typing import Optional

Coordinate = tuple[int, int] | np.ndarray
Corners = tuple[Coordinate, Coordinate, Coordinate, Coordinate]


def find_largest_rectangle(matrix: np.ndarray) -> Optional[np.ndarray]:
    """Find the largest rectangle in a binary matrix.

    :param matrix: The binary matrix.
    :return: The corners of the largest rectangle (tl, tr, br, bl).
    """
    if not np.any(matrix):
        return None

    max_area = 0
    top_left = (0, 0)
    bottom_right = (0, 0)

    rows, cols = matrix.shape[:2]

    for i in range(rows):
        for j in range(cols):
            if matrix[i, j] == 0:
                continue

            for x in range(i, rows):
                for y in range(j, cols):
                    if matrix[x][y] == 1 and matrix[i][y] == 1 and matrix[x][j] == 1:
                        area = (x - i + 1) * (y - j + 1)
                        if area > max_area:
                            max_area = area
                            top_left = (i, j)
                            bottom_right = (x, y)

    top_right = (top_left[0], bottom_right[1])
    bottom_left = (bottom_right[0], top_left[1])

    return np.array([top_left, top_right, bottom_right, bottom_left])


def euclidean_distance(p1: np.ndarray | Coordinate, p2: np.ndarray | Coordinate) -> float:
    """Calculate the Euclidean distance between two points.

    :param p1: The first point.
    :param p2: The second point.
    :return: The Euclidean distance between the two points.
    """
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def corners_to_grid(corners: np.ndarray, ids: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Convert corners and ids to a grid.

    :param corners: An array of corners.
    :param ids: An array of ids for each corner.
    :param shape: The shape of the board (w, h).
    :return: The grid of corners.
    """
    w, h = np.subtract(shape, 1)
    grid = np.zeros((h, w, 2), dtype=np.float32)

    for corner, corner_id in zip(corners[:, 0], ids[:, 0]):
        col = corner_id % w
        row = corner_id // w

        grid[row, col] = corner

    return grid


def get_transformed_corners(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """Get the transformed corners of the image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :return: The transformed corners of the image.
    """
    h, w = shape
    src_points = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32).reshape(-1, 1, 2)
    dst_points = cv2.perspectiveTransform(src_points, matrix)

    min_x = np.amin(dst_points[:, 0, 0])
    min_y = np.amin(dst_points[:, 0, 1])
    max_x = np.amax(dst_points[:, 0, 0])
    max_y = np.amax(dst_points[:, 0, 1])
    return int(min_x), int(min_y), int(max_x), int(max_y)


def get_transformed_shape(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[int, int]:
    """Get the transformed shape of the image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :return: The transformed shape of the image.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, shape)
    return int(max_y - min_y), int(max_x - min_x)


def get_scale_factor(matrix: np.ndarray, shape: tuple[int, int], max_height: int, max_width: int) -> float:
    """Get the scale factor for the perspective matrix.

    :param matrix: The perspective matrix.
    :param shape: The shape of the image.
    :param max_height: The maximum height of the new image.
    :param max_width: The maximum width of the new image.
    :return: The scale factor for the perspective matrix.
    """
    new_h, new_w = get_transformed_shape(matrix, shape)
    return min(max_width / new_w, max_height / new_h)


def adjust_perspective(matrix: np.ndarray, shape: tuple[int, int]) -> tuple[np.ndarray, int, int]:
    """Adjust the perspective matrix to fit within the new image.

    :param matrix: The perspective matrix.
    :param shape: The shape of the old image.
    :return: The adjusted perspective matrix and the new shape of the image.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, shape)

    new_w = int(max_x - min_x)
    new_h = int(max_y - min_y)

    adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
    adjusted_matrix = np.dot(adjusted_matrix, matrix)

    return adjusted_matrix, new_w, new_h


def scale_matrix(matrix: np.ndarray, scale_factor: float) -> np.ndarray:
    """Scale the perspective matrix.

    :param matrix: The perspective matrix.
    :param scale_factor: The scale factor.
    :return: The scaled perspective matrix.
    """
    scaled_matrix = np.array([[scale_factor, 0, 0], [0, scale_factor, 0], [0, 0, 1]], dtype=np.float32)
    return np.dot(scaled_matrix, matrix)


def get_board_shape() -> tuple[int, int]:
    """Get the shape of the ChArUco board.

    :return: The shape of the ChArUco board.
    """
    return Config.BOARD_WIDTH, Config.BOARD_HEIGHT


def get_charuco_detector() -> cv2.aruco.CharucoDetector:
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


def get_slope(p1: Coordinate, p2: Coordinate, width: int) -> np.ndarray:
    """Get the slope of a line.

    :param p1: The first point.
    :param p2: The second point.
    :param width: The width of the image.
    :return: The horizontal and vertical change of the line.
    """
    return np.array([p2[0] - p1[0], p2[1] - p1[1]]) / width


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


def get_src_grid(detector: cv2.aruco.CharucoDetector, image: np.ndarray) -> np.ndarray:
    """Get the source grid for the image.

    :param detector: The ChArUco detector.
    :param image: The image to get the grid for.
    :return: The source grid for the image.
    """
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
    if charuco_corners is None or len(charuco_corners) < 4:
        raise ValueError("The ChArUco board was not detected")

    return corners_to_grid(charuco_corners, charuco_ids, get_board_shape())


def get_dst_grid(h_change: float, v_change: float) -> np.ndarray:
    """Calculate the destination grid for the image.

    :param h_change: The horizontal change per square.
    :param v_change: The vertical change per square.
    :return: The destination grid for the image.
    """
    w, h = np.subtract(get_board_shape(), 1)
    dst_grid = np.zeros((h, w, 2), dtype=np.float32)

    for i in range(h):
        for j in range(w):
            x = j * v_change + i * h_change
            y = (w - j) * h_change + i * v_change

            dst_grid[i, j] = [x, y]

    return dst_grid
