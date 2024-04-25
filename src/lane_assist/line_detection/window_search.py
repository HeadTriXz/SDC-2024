import itertools
import numpy as np
import scipy

from collections.abc import Iterable
from config import config
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window


def window_search(
        filtered_img: np.ndarray, window_count: int, windows: Iterable[Window], window_height: int,
        stopline: bool = False
) -> list[Line]:
    """Search for the windows in the image.

    :param filtered_img: The filtered image.
    :param window_count: The number of windows to search for.
    :param windows: The windows to search for.
    :param window_height: The height of the windows.
    :param stopline: Whether we are searching for a stopline.
    :return: The lines in the image.
    """
    img_center = filtered_img.shape[1] // 2

    for _ in range(window_count):
        running_windows = [window for window in windows if not window.collided]

        for window_0, window_1 in list(itertools.combinations(running_windows, 2)):
            if window_0.collides(window_1):
                __kill_windows(window_0, window_1, img_center)

        for window in running_windows:
            # Get the new sides of the window.
            top = min(max(window.y - window_height, 0), filtered_img.shape[0])
            bottom = min(max(window.y, 0), filtered_img.shape[0])
            left = min(max(window.x - int(window.margin), 0), filtered_img.shape[1])
            right = min(max(window.x + int(window.margin), 0), filtered_img.shape[1])

            non_zero_count = np.sum(filtered_img[top:bottom, left:right]) // 255
            if non_zero_count < config.lane_assist.line_detection.pixels_in_window:
                window.move(window.x, top, False)
                continue

            center_masses = scipy.ndimage.center_of_mass(filtered_img[top:bottom, left:right])
            if center_masses[0] != center_masses[0]:
                # If they are not equal to themselves, they are NaN.
                # This should never happen, but if it does, we move the window up.
                window.move(window.x, top, False)
                continue

            if isinstance(center_masses, list):
                center_masses = center_masses[-1]

            window.move(int(center_masses[1]) + left, top)

    line_type = LineType.STOP if stopline else None
    return [
        Line(window.points, window_height, line_type=line_type)
        for window in windows
        if window.point_index >= 5 or stopline
    ]


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
