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

    while not __window_at_bounds(image, window):
        top, bottom, left, right = window.get_borders(image.shape)

        chunk = image[top:bottom, left:right]
        non_zero = np.transpose(np.nonzero(chunk))

        # Move the window if there are not enough points in it
        if len(non_zero) < config["line_detection"]["pixels_in_window"]:
            __move_no_points(window)
            continue

        # Calculate the new position of the window
        target = max(0, min(image_center - left, window.shape[1]))
        offset = center_of_masses(chunk, target, config["line_detection"]["pixels_in_window"])
        if offset is None:
            __move_no_points(window)
            continue

        x_shift, y_shift = offset
        if stop_line:
            y_shift = 0

        new_pos = (left + x_shift, top + y_shift)

        # Kill the window if we suddenly change direction.
        if window.not_found >= 3 and window.point_count > len(window.directions):
            angle_diff = __get_angle(window, new_pos)
            if angle_diff > config["line_detection"]["thresholds"]["max_angle_difference"]:
                break

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
    x_diff, y_diff = np.subtract(window.last_point, new_pos)
    curr_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

    # Get the angle of the line
    x_diff, y_diff = window.directions.mean(axis=0)
    prev_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

    return abs(prev_direction - curr_direction)


def __move_no_points(window: Window) -> None:
    """Move the window if there are no points in it.

    :param window: The window to move.
    """
    x_shift = 0
    y_shift = -window.shape[0]

    if window.point_count >= 3:
        x_shift, y_shift = window.directions.mean(axis=0)

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
