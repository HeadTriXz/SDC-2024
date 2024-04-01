import dataclasses
import itertools
from typing import Any

import numpy as np
import scipy

from config import config
from lane_assist.line_detection.line import Line
from lane_assist.line_detection.window import Window

# REPLACED WITH THE VALUES IN GLOBALS
THRESHOLDS = config.lane_assist.line_detection.thresholds


def window_search(
    img: np.ndarray, window_count: int, _pixels_per_window: int = 1, window_width: int = 30
) -> tuple[list[Line], list[Any]]:
    """Get the lines in the image using the sliding window algorithm.

    first we take a histogram of the x axis. this is done to filter pout the zebra crossings.
    after that we take a histogram of the other axis. this is used for detecting the starting points of the line.
    after that we detect the 2 lines closest to the center. this will be the starting point for the windows.
    if we detect a couple od

    Todo:
    ----
    - add support for multiple horizontal lines
    - improve support for stop lines
        - fix position of stop line

    Parameter
    ---------
    :param window_width: the width of the window to use for the window search
    :param pixels_per_window: the minimum amount of pixels needed in the window to be considered part of a line
    :parameter img: the image to get the lines from.
    :parameter window_count: the amount of windows to check in the image

    """
    # filter out the zebra crossing and stoplines from the image
    img, filtered_peaks = __filter_image(img)

    # get the actual start position of the image.
    # this is different from the height of the image because we might have a zebra crossing
    # or a stopline in the image. in that case, we want to start above the zebra crossing or stopline
    start_position = img.shape[0] - 1
    if len(filtered_peaks) > 0:  # check if we have any peaks
        lowest_end_index = np.argmax([peak.right for peak in filtered_peaks])

        if filtered_peaks[lowest_end_index].right > img.shape[0] - 10:
            start_position = int(filtered_peaks[lowest_end_index].center) - 10

    # create a histogram of the image to find the lines.
    # we only use the bottom half because that should be where the lines are.
    # to get the lines we will detect and get the peaks

    #  get the actual height of what the histogram should be taken of
    weights = np.linspace(0.5, 1, img.shape[0] // 2)
    pixels = img[img.shape[0] // 2 :, :]
    pixels = np.multiply(pixels, weights[:, np.newaxis])
    histogram = np.sum(pixels, axis=0)

    merged_peaks = scipy.signal.find_peaks(histogram, distance=config.lane_assist.line_detection.line_width * 2)[0]

    window_height = img.shape[0] // window_count
    window_count = start_position // window_height
    windows = [Window(center, start_position, window_width // 2, window_count) for center in merged_peaks]

    lines = __window_search(img, window_count, windows)
    stoplines = [
        peak.center for peak in filtered_peaks if peak.width < config.lane_assist.line_detection.line_width * 2
    ]

    return lines, stoplines


def __kill_windows(window: Window, other_window: Window, img_center: int) -> None:
    """Kill the window that is furthest from the center of the image."""
    if (
        window.x - window.margin < other_window.x + other_window.margin
        and window.x + window.margin > other_window.x - other_window.margin
    ):
        if window.found_in_previous and other_window.found_in_previous:
            # kill the one furthest from the center
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
    """A class to represent a peak in the histogram."""

    center: int
    width: int
    left: int
    right: int


def __filter_image(img: np.ndarray) -> (np.ndarray, list[HistogramPeak]):
    """Filter the image based on the axis."""
    # only take the center 2/3 of the image
    pixels = img[:, img.shape[0] // 3 : img.shape[0] // 3 * 2]
    histogram = np.sum(pixels, axis=1)

    filter_peaks = scipy.signal.find_peaks(
        histogram,
        height=THRESHOLDS.zebra_crossing / 3,
        distance=config.lane_assist.line_detection.line_width * 4,
    )[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(histogram, filter_peaks, rel_height=0.98)

    peaks = [
        HistogramPeak(center, width, left, right)
        for center, width, left, right in zip(filter_peaks, widths, lefts, rights)
    ]

    for peak in peaks:
        img[int(peak.left) : int(peak.right)] = 0

    return img, peaks


def __window_search(filtered_img: np.ndarray, window_count: int, windows: list[Window]) -> list[Line]:
    window_height = filtered_img.shape[0] // window_count  # get the height of the windows based on the amount we want.

    for _ in range(window_count):
        running_windows = [window for window in windows if not window.collided]

        for window_0, window_1 in list(itertools.combinations(running_windows, 2)):
            if window_0.collides(window_1):
                __kill_windows(window_0, window_1, filtered_img.shape[1] // 2)

        # do the window search
        for window in running_windows:
            # get the new sides of the window.
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

            # draw the window
            window.move(int(center_masses[1]) + left, top)

    return [Line(np.array(window.points[:-3]), window_height) for window in windows if len(window.points) > 5]
