import math
import numpy as np

from collections.abc import Iterable

from src.config import config
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.window import Window
from src.utils.center_of_masses import center_of_masses


def process_window(image: np.ndarray, window: Window, stop_line: bool) -> Line | None:
    """Process the window.

    :param image: The image to process.
    :param window: The window to process.
    :param stop_line: Whether we are searching for a stop line.
    :return: The processed window.
    """
    image_center = image.shape[1] // 2
    attempts_left = 3
    attempts_reset_after = 10

    while not __window_at_bounds(image, window):
        top, bottom, left, right = window.get_borders(image.shape)

        chunk = image[top:bottom, left:right]
        non_zero = np.transpose(np.nonzero(chunk))

        # Move the window if there are not enough points in it
        if len(non_zero) < config["line_detection"]["window"]["min_pixels"]:
            __move_no_points(window)
            continue

        # Calculate the new position of the window
        ref_point = __get_ref_point(window)
        target_point = __get_target_point(window, ref_point, image_center)

        offset = center_of_masses(chunk, target_point, config["line_detection"]["window"]["min_pixels"])
        if offset is None:
            __move_no_points(window)
            continue

        x_shift, y_shift = offset
        if stop_line:
            y_shift = 0

        new_pos = (left + x_shift, top + y_shift)

        # Kill the window if we suddenly change direction.
        if window.point_count > 1:
            angle_diff = __get_angle(window, new_pos)
            if window.not_found >= 3 and angle_diff > config["line_detection"]["max_angle_difference"]:
                break

            is_junction = angle_diff > config["line_detection"]["max_angle_junction"]
            if window.not_found == 0 and is_junction and attempts_left > 0:
                x_diff, y_diff = window.directions[1:].mean(axis=0)
                attempts_left -= 1

                window.move_back()
                window.move(int(window.x + x_diff), int(window.y + y_diff), False)
                continue

        if attempts_reset_after == 0:
            attempts_left = 3
            attempts_reset_after = 10

        attempts_reset_after -= 1
        window.move(new_pos[0], new_pos[1])

    # Check if we have enough points to make a line
    if window.point_count == 0 or (window.point_count < 5 and not stop_line):
        return None

    line_type = LineType.STOP if stop_line else None
    return Line(window.points, window.shape[0], line_type)


def window_search(image: np.ndarray, windows: Iterable[Window], stop_line: bool = False) -> list[Line]:
    """Search for the windows in the image.

    :param image: The filtered image.
    :param windows: The windows to search for.
    :param stop_line: Whether we are searching for a stop line.
    :return: The lines in the image.
    """
    lines = []
    for window in windows:
        line = process_window(image, window, stop_line)
        if line is not None:
            lines.append(line)

    return lines


def __get_angle(window: Window, new_pos: tuple[int, int]) -> float:
    """Get the angle between the last point and the new position.

    :param window: The window.
    :param new_pos: The new position.
    :return: The angle between the last point and the new position.
    """
    # Get the angle of the last point to the current point.
    x_diff, y_diff = np.subtract(new_pos, window.last_point)
    curr_direction = math.atan2(y_diff, x_diff) * 180 / np.pi

    # Get the angle of the line
    x_diff, y_diff = window.directions.sum(axis=0)
    prev_direction = math.atan2(y_diff, x_diff) * 180 / np.pi

    return abs(prev_direction - curr_direction)


def __get_ref_point(window: Window) -> int:
    """Get the reference point for the window.

    :param window: The window to get the reference point for.
    :return: The reference point.
    """
    ref_point = window.x
    if window.point_count > 0:
        ref_point = window.first_point[0]

    return ref_point + window.margin


def __get_target_point(window: Window, ref_point: int, image_center: int) -> int:
    """Get the target point for the window.

    :param window: The window to get the target point for.
    :param ref_point: The reference point.
    :param image_center: The center of the image.
    :return: The target point.
    """
    if ref_point < image_center:
        return window.shape[1]

    return 0


def __move_no_points(window: Window) -> None:
    """Move the window if there are no points in it.

    :param window: The window to move.
    """
    x_shift = 0
    y_shift = -window.shape[0]

    if window.point_count >= len(window.directions):
        x_shift, y_shift = window.directions.mean(axis=0)
    elif window.point_count > 1:
        x_shift, y_shift = window.directions.sum(axis=0)
        y_shift = max(-window.shape[0], y_shift)

    window.move(window.x + x_shift, window.y + y_shift, False)


def __window_at_bounds(image: np.ndarray, window: Window) -> bool:
    """Check if the window is at the bounds of the image.

    :param image: The image to check.
    :param window: The window to check.
    :return: Whether the window is at the bounds.
    """
    return (
        window.y - window.shape[0] < 0
        or window.x - window.margin // 3 < 0
        or window.x + window.margin // 3 >= image.shape[1]
    )
