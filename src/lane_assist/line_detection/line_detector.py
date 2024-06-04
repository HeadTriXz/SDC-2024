import cv2
import numpy as np
import scipy

from collections.abc import Callable
from typing import Any

from src.calibration.data import CalibrationData
from src.config import config
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.window import Window
from src.lane_assist.line_detection.window_search import window_search
from src.lane_assist.preprocessing.image_filters import morphex_filter
from src.utils.other import euclidean_distance, get_border_of_points


def filter_lines(lines: list[Line], position: tuple[int, int]) -> list[Line]:
    """Get the lines between the solid lines closest to each side of the starting point.

    :param lines: The lines to filter.
    :param position: Our position in the image.
    :return: The filtered lines.
    """
    sorted_lanes = sorted(lines, key=lambda line: euclidean_distance(line.scan_data[0], position))

    closest_left = None
    closest_right = None

    for line in sorted_lanes:
        if closest_left is not None and closest_right is not None:
            break

        if line.line_type != LineType.SOLID:
            continue

        if closest_left is None and line.points[0][0] < position[0]:
            closest_left = line

        if closest_right is None and line.points[0][0] > position[0]:
            closest_right = line

    start_idx = 0
    stop_idx = len(lines)

    if closest_left is not None:
        start_idx = lines.index(closest_left)

    if closest_right is not None:
        stop_idx = lines.index(closest_right) + 1

    return lines[start_idx:stop_idx]


def get_lines(image: np.ndarray, calibration: CalibrationData) -> list[Line]:
    """Get the lines in the image.

    :param image: The image to get the lines from.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :return: The lines in the image.
    """
    # Filter the image. This is done in place and will be used to remove zebra crossings.
    if config["line_detection"]["filtering"]["active"]:
        filter_mask = cv2.bitwise_not(morphex_filter(image, calibration))
        image = cv2.bitwise_and(image, filter_mask)

    # Create histogram to find the start of the lines.
    # This is done by weighting the pixels using a logspace.
    pixels = image[image.shape[0] // 2:, :]
    pixels = np.multiply(pixels, np.logspace(0, 1, pixels.shape[0])[:, np.newaxis])
    histogram = np.sum(pixels, axis=0)

    return __get_lines(image, histogram, calibration)[0]


def get_stop_lines(image: np.ndarray, lines: list[Line], calibration: CalibrationData) -> list[Line]:
    """Get the stop lines in the image.

    :param lines: The lines in the image.
    :param image: The image to get the stop lines from.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :return: The stop lines in the image.
    """
    # Get the bounding box of the lines.
    points = __lines_to_points(lines)
    if len(points) == 0:
        return []

    min_x, min_y, max_x, max_y = get_border_of_points(points)

    min_dist = calibration.get_pixels(config["line_detection"]["filtering"]["min_distance"])
    max_y = min(max_y, image.shape[0] - min_dist)

    # Create a new image. This is the bounding box rotated 90 degrees clockwise.
    new_img = image[min_y:max_y, min_x:max_x]
    new_img = cv2.rotate(new_img, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Get the lines in the image.
    histogram = np.sum(new_img, axis=0)
    rotated_lines, window_height = __get_lines(new_img, histogram, calibration, True)

    min_windows = calibration.get_pixels(2.5) // window_height
    max_windows = calibration.get_pixels(3.5) // window_height

    rotated_lines = __filter_stop_lines(rotated_lines, window_height, min_windows, max_windows)
    return __rotate_lines(rotated_lines)


def __filter_stop_lines(lines: list[Line], window_height: int, minimum_points: int, max_points: int) -> list[Line]:
    """Filter the stop lines to be actual stop lines.

    :param lines: The lines to filter.
    :param window_height: The height of the window.
    :param minimum_points: The minimum number of points needed to be considered a stop line.
    :param max_points: The maximum number of points needed to be considered a stop line.
    :return: The filtered lines.
    """
    filtered_lines = []
    for line in lines:
        distances = np.linalg.norm(np.diff(line.points, axis=0), axis=1)

        start, stop = __longest_sequence(distances, lambda x: window_height + 2 > x > window_height - 2)
        if minimum_points < stop - start < max_points:
            filtered_lines.append(Line(line.points[start:stop], line_type=LineType.STOP))

    return filtered_lines


def __get_lines(
    image: np.ndarray, histogram: np.ndarray, calibration: CalibrationData, stop_line: bool = False
) -> tuple[list[Line], int]:
    """Get the lines in the image.

    This function is a wrapper for the window search function. It calculates the window sizes and the number of windows
    needed to cover the image. It then calls the window search function to get the lines.

    :param image: The image to get the lines from.
    :param histogram: The histogram of the image.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :return: The lines in the image and the height of the windows.
    """
    mean = np.mean(histogram)
    std = np.std(histogram)
    threshold = mean + std

    max_width = calibration.get_pixels(config["line_detection"]["window"]["max_width"])
    window_width = calibration.get_pixels(config["line_detection"]["window"]["min_width"])
    window_height = calibration.get_pixels(config["line_detection"]["window"]["height"])
    window_shape = (window_height, window_width)

    peaks = scipy.signal.find_peaks(histogram, height=threshold, distance=window_width * 2, rel_height=0.9)[0]
    windows = [Window(center, image.shape[0], window_shape, max_width) for center in peaks]
    lines = window_search(image, windows, stop_line)

    return lines, window_height


def __longest_sequence(items: np.ndarray, condition: Callable[[Any], bool]) -> tuple[int, int]:
    """Get the longest subsequence of numbers that satisfy the condition.

    :param items: The boolean array to get the subsequence from.
    :param condition: The condition to satisfy.
    :return: The start and end index of the subsequence.
    """
    bools = np.array([condition(item) for item in items])
    idx = np.where(np.diff(np.hstack(([False], bools, [False]))))[0].reshape(-1, 2)
    if len(idx) == 0:
        return 0, 0

    idx = idx[np.argmax(np.diff(idx, axis=1)), :]
    return idx[0], idx[1] + 1


def __lines_to_points(lines: list[Line]) -> np.ndarray:
    """Convert the lines to a numpy array of points.

    :param lines: The lines to convert.
    :return: The points of the lines.
    """
    return np.concatenate([line.points for line in lines], dtype=np.int32)


def __rotate_lines(lines: list[Line]) -> list[Line]:
    """Rotate the lines 90 degrees clockwise.

    :param lines: The lines to rotate.
    :return: The rotated lines.
    """
    rotated_lines = []
    for line in lines:
        line = Line(line.points[:, [1, 0]], line_type=line.line_type)
        rotated_lines.append(line)

    return rotated_lines
