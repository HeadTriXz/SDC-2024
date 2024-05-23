import numpy as np

from collections.abc import Iterable

from src.config import config
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.window import Window


def process_window(image: np.ndarray, window: Window, stop_line: bool) -> Line | None:
    """Process the window.

    :param image: The image to process.
    :param window: The window to process.
    :param stop_line: Whether we are searching for a stop line.
    :return: The processed window.
    """
    while True:
        if __window_at_bounds(image, window):
            break

        top, bottom, left, right = window.get_borders(image.shape)
        non_zero = np.argwhere(image[top:bottom, left:right])

        # Move the window if there are not enough points in it
        if len(non_zero) < config["line_detection"]["pixels_in_window"]:
            window.move(window.x, top, False)
            continue

        # Get the average position of the points
        y_shift, x_shift = np.mean(non_zero, axis=0).astype(int)
        if stop_line:
            y_shift = 0

        new_pos = [left + x_shift, top + y_shift]

        # Kill the window if we suddenly change direction.
        if window.not_found >= 3 and window.point_count > len(window.directions):
            # Get the angle of the last point to the current point.
            x_diff, y_diff = np.subtract(window.points[-1], new_pos)
            current_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

            # Get the angle of the line
            x_diff, y_diff = window.directions.mean(axis=0)
            prev_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

            angle_diff = abs(prev_direction - current_direction)
            if angle_diff > config["line_detection"]["thresholds"]["max_angle_difference"]:
                break

        window.move(new_pos[0], new_pos[1])

    if window.point_count == 0:
        return None

    if window.point_count < 5 and not stop_line:
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
