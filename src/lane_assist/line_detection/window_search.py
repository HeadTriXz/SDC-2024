import dataclasses
import itertools
import numpy as np
import scipy

from config import config
from lane_assist.line_detection.line import Line
from lane_assist.line_detection.window import Window
from typing import Any

# REPLACED WITH THE VALUES IN GLOBALS
THRESHOLDS = config.lane_assist.line_detection.thresholds


def window_search(img: np.ndarray, window_count: int, window_width: int = 30) -> tuple[list[Line], list[Any]]:
    """Get the lines in the image using the sliding window algorithm.

    First we take a histogram of the x-axis. This is done to filter pout the zebra crossings.
    After that we take a histogram of the other axis. This is used for detecting the starting points of the line.
    After that we detect the 2 lines closest to the center. This will be the starting point for the windows.
    If we detect a couple of lines, we will take the 2 lines closest to the center of the image.

    :param img: The image to search the lines in.
    :param window_count: The amount of windows to check in the image.
    :param window_width: The width of the window to use for the window search.
    :return: The lines in the image.
    """
    # TODO:
    # - add support for multiple horizontal lines
    # - improve support for stop lines
    #    - fix position of stop line

    img, filtered_peaks = __filter_image(img)

    # Get the actual start position of the image.
    # This is different from the height of the image because we might have a zebra crossing
    # or a stop line in the image. In that case, we want to start above the zebra crossing or stop line.
    start_position = img.shape[0] - 1
    if len(filtered_peaks) > 0:
        lowest_end_index = np.argmax([peak.right for peak in filtered_peaks])

        if filtered_peaks[lowest_end_index].right > img.shape[0] - 10:
            start_position = int(filtered_peaks[lowest_end_index].center) - 10

    # Create a histogram of the image to find the lines.
    # We only use the bottom half because that should be where the lines are.
    # To get the lines, we will detect and get the peaks in the histogram.
    weights = np.linspace(0.5, 1, img.shape[0] // 2)

    pixels = img[img.shape[0] // 2:, :]
    pixels = np.multiply(pixels, weights[:, np.newaxis])
    histogram = np.sum(pixels, axis=0)

    # Find the peaks in the histogram
    min_distance = 2 * config.lane_assist.line_detection.line_width
    merged_peaks = scipy.signal.find_peaks(histogram, distance=min_distance)[0]

    window_height = img.shape[0] // window_count
    window_count = start_position // window_height
    windows = [Window(center, start_position, window_width // 2, window_count) for center in merged_peaks]

    lines = __window_search(img, window_count, windows)
    stop_lines = [peak.center for peak in filtered_peaks if peak.width < min_distance]

    return lines, stop_lines


def __kill_windows(window: Window, other_window: Window, img_center: int) -> None:
    """Kill the window that is furthest from the center of the image.

    :param window: The first window.
    :param other_window: The second window.
    :param img_center: The center of the image (x-axis).
    """
    if (
        window.x - window.margin < other_window.x + other_window.margin and
        window.x + window.margin > other_window.x - other_window.margin
    ):
        if window.found_in_previous and other_window.found_in_previous:
            # Kill the one furthest from the center.
            if abs(window.x - img_center) < abs(other_window.x - img_center):
                other_window.collided = True
            else:
                window.collided = True
        elif window.found_in_previous:
            other_window.collided = True
        elif other_window.found_in_previous:
            window.collided = True


@dataclasses.dataclass
class HistogramPeak:
    """A class to represent a peak in the histogram.

    Attributes
    ----------
        center: The center of the peak.
        width: The width of the peak.
        left: The left side of the peak.
        right: The right side of the peak.

    """

    center: int
    width: int
    left: int
    right: int


def __filter_image(img: np.ndarray) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param img: The image to filter.
    :return: The filtered image and the peaks.
    """
    third = img.shape[0] // 3
    pixels = img[:, third:2 * third]

    histogram = np.sum(pixels, axis=1)
    filter_peaks = scipy.signal.find_peaks(
        histogram,
        height=THRESHOLDS.zebra_crossing / 3,
        distance=config.lane_assist.line_detection.line_width * 4,
    )[0]

    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram,
        filter_peaks,
        rel_height=config.lane_assist.line_detection.filtering.rel_height
    )

    peaks = list(map(lambda params: HistogramPeak(*params), zip(filter_peaks, widths, lefts, rights)))
    for peak in peaks:
        img[int(peak.left):int(peak.right)] = 0

    return img, peaks


def __window_search(filtered_img: np.ndarray, window_count: int, windows: list[Window]) -> list[Line]:
    """Search for the windows in the image.

    :param filtered_img: The filtered image.
    :param window_count: The amount of windows to search for.
    :param windows: The windows to search for.
    :return: The lines in the image.
    """
    # Get the height of the windows based on the amount of windows we want.
    window_height = filtered_img.shape[0] // window_count

    for _ in range(window_count):
        running_windows = [window for window in windows if not window.collided]

        for window_0, window_1 in list(itertools.combinations(running_windows, 2)):
            if window_0.collides(window_1):
                __kill_windows(window_0, window_1, filtered_img.shape[1] // 2)

        for window in running_windows:
            # Get the new sides of the window.
            top = window.y - window_height
            bottom = window.y
            left = window.x - int(window.margin)
            right = window.x + int(window.margin)

            non_zero_count = np.sum(filtered_img[top:bottom, left:right])
            if non_zero_count < config.lane_assist.line_detection.pixels_in_window:
                window.move(window.x, top, False)
                continue

            center_masses = scipy.ndimage.center_of_mass(filtered_img[top:bottom, left:right])
            if center_masses[0] != center_masses[0]:
                continue

            if isinstance(center_masses, list):
                center_masses = center_masses[-1]

            window.move(int(center_masses[1]) + left, top)

    return [Line(np.array(window.points[:-3]), window_height) for window in windows if len(window.points) > 5]
