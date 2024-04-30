from collections.abc import Callable
from typing import Any

import cv2
import numpy as np
import scipy

from config import config
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window
from lane_assist.line_detection.window_search import window_search
from lane_assist.preprocessing.image_filters import basic_filter
from lane_assist.preprocessing.utils.corners import get_border_of_points
from utils.calibration_data import CalibrationData


def filter_lines(lines: list[Line], starting_point: int) -> list[Line]:
    """Get the lines between the solid lines closest to each side of the starting point.

    :param lines: The lines to filter.
    :param starting_point: The starting point.
    :return: The filtered lines.
    """
    i = 0
    j = 0

    while i < len(lines):
        # check if we are after the starting point
        if lines[i].points[0][0] >= starting_point and lines[i].line_type == LineType.SOLID:
            # back up until we find a solid line
            j = i
            while j > 0:
                j -= 1
                if lines[j].line_type == LineType.SOLID:
                    break
            break
        i += 1

    return lines[j : i + 1]


def get_lines(image: np.ndarray, calibration: CalibrationData) -> list[Line]:
    """Get the lines in the image.

    :param image: The image to get the lines from.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :param filter_image: Whether to filter the image or not.
    :return: The lines in the image.
    """
    # Dilate the image to make the lines thicker and more solid. Then threshold the image.
    cv2.dilate(image, np.ones((3, 3), np.uint8), image)
    cv2.threshold(image, config.image_manipulation.white_threshold, 255, cv2.THRESH_BINARY, image)

    # Filter the image. This is done in place and will be used to remove zebra crossings.
    if config.lane_assist.line_detection.filtering.active:
        basic_filter(image, calibration)
        # filter_small_clusters(image)  # noqa: ERA001

    # create histogram to find the start of the lines
    pixels = image[image.shape[0] // 2 :, :]
    pixels = np.multiply(pixels, np.linspace(0, 1, pixels.shape[0])[:, np.newaxis])
    histogram = np.sum(pixels, axis=0)

    # Window search options
    lines = __get_lines(image, histogram, calibration)[0]

    # Remove the first and last point. These can be in non-useful locations.
    # For example, the start of the line (done for detection of stoplines)
    # or on a different line in a turn.
    for line in lines:
        line.points = line.points[1:-1]

    return lines


def get_stoplines(image: np.ndarray, lines: list[Line], calibration: CalibrationData) -> list[Line]:
    """Get the stop lines in the image.

    :param lines: The lines in the image.
    :param image: The image to get the stop lines from.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :return: The stop lines in the image.
    """
    # Get the bounding box of the lines.
    points = __lines_to_points(lines)
    x_min, x_max, y_min, y_max = get_border_of_points(points)

    # Create a new image. This is the bounding box rotated 90 degrees clockwise.
    new_img = image[y_min:y_max, x_min:x_max]
    new_img = cv2.rotate(new_img, cv2.ROTATE_90_CLOCKWISE)

    # Get the lines in the image.
    histogram = np.sum(new_img, axis=0)
    rotated_lines, window_height = __get_lines(new_img, histogram, calibration, True)

    lines = []
    # rotate the lines to its original position
    for line in rotated_lines:
        points = np.flip(line.points, axis=1)
        points[:, 0] = x_min + points[:, 0]
        points[:, 1] = y_max - points[:, 1]

        lines.append(Line(points, line_type=LineType.STOP))

    # get the number of windows needed to be at least 2.5 meters long. a stopline will be 3 meters long
    min_windows = int(2.5 * calibration.pixels_per_meter) // window_height
    max_windows = int(3.5 * calibration.pixels_per_meter) // window_height
    return filter_stoplines(lines, window_height, min_windows, max_windows)
    # return filter_stoplines(lines, window_height, 8, 999)  # noqa: ERA001 Simulator has no real calibration


def filter_stoplines(lines: list[Line], window_height: int, minimum_points: int, max_points: int) -> list[Line]:
    """Filter the stoplines to be actual stoplines.

    :param lines: The lines to filter.
    :param window_height: The height of the window.
    :param minimum_points: The minimum number of points needed to be considered a stopline.
    :param max_points: The maximum number of points needed to be considered a stopline.
    :return: The filtered lines.
    """
    filtered_lines = []
    for line in lines:
        gaps = np.diff(line.points[:, 0])
        start, stop = __longest_sequence(gaps, lambda x: window_height + 2 > -x > window_height - 2)
        if minimum_points < stop - start < max_points:
            filtered_lines.append(Line(line.points[start:stop], line_type=LineType.STOP))

    return filtered_lines


def __get_lines(
    image: np.ndarray, histogram: np.ndarray, calibration: CalibrationData, stopline: bool = False
) -> tuple[list[Line], int]:
    """Get the lines in the image.

    This function is a wrapper for the window search function. It calculates the window sizes and the number of windows
    needed to cover the image. It then calls the window search function to get the lines.

    :param image: The image to get the lines from.
    :param histogram: The histogram of the image.
    :param calibration: The calibration data of the stitching, used for calculating the window sizes.
    :return: The lines in the image and the height of the windows.
    """
    std = np.std(histogram)
    window_width = int(calibration.pixels_per_meter * config.lane_assist.line_detection.window_sizing.width)
    window_height = int(calibration.pixels_per_meter * config.lane_assist.line_detection.window_sizing.height)
    window_count = image.shape[0] // window_height

    peaks = scipy.signal.find_peaks(histogram, height=std, distance=window_width * 2)[0]
    windows = [Window(int(center), image.shape[0], window_width // 2, window_count) for center in peaks]
    lines = window_search(image, window_count, windows, image.shape[0] // window_count, stopline)
    return lines, window_height


def __longest_sequence(items: np.ndarray, condition: Callable[[Any], bool]) -> tuple[int, int]:
    """Get the longest subsequence of numbers that satisfy the condition.

    :param items: The boolean array to get the subsequence from.
    :param condition: The condition to satisfy.
    :return: The start and end index of the subsequence.
    """
    # use numpy to get the start and end of the longest consecutive True sequence
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
