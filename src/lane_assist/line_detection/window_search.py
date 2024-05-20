import numpy as np

from collections.abc import Iterable

from src.config import config
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.window import Window


def process_window(image: np.ndarray, window: Window, window_height: int, stop_line: bool) -> Line | None:
    """Process the window.

    :param image: The image to process.
    :param window: The window to process.
    :param window_height: The height of the window.
    :param stop_line: Whether we are searching for a stop line.
    :return: The processed window.
    """
    while True:
        top = max(0, min(int(window.y) - window_height, image.shape[0]))
        bottom = max(0, min(int(window.y), image.shape[0]))
        left = max(0, min(int(window.x) - int(window.margin), image.shape[1]))
        right = max(0, min(int(window.x) + int(window.margin), image.shape[1]))

        # Check if the window is at the edge of the image. If it is, we should stop.
        if __window_at_bounds(image, window, window_height):
            break

        # We should go up if there are no pixels in the window.
        non_zero = np.argwhere(image[top:bottom, left:right])
        if len(non_zero) < config.line_detection.pixels_in_window:
            __shift_no_points(window, window_height)
            continue

        y_shift, x_shift = np.mean(non_zero, axis=0).astype(int)

        # Make sure we will move at least a certain amount. If not, we handle it as no pixels in the window.
        # This prevents points from clumping together at the end of lines.
        if np.linalg.norm([x_shift, y_shift]) < window_height * config.line_detection.min_window_shift:
            __shift_no_points(window, window_height)
            continue

        # Kill the window if we suddenly change direction.
        if window.not_found >= 3 and window.point_count > len(window.directions):
            # Get the angle of the last point to the current point.
            x_diff, y_diff = np.subtract(window.points[-1], [window.x, window.y])
            current_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

            # get the angle of the line
            x_diff, y_diff = window.directions.mean(axis=0)
            prev_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi)

            angle_diff = abs(prev_direction - current_direction)
            if angle_diff > config.line_detection.thresholds.max_angle_difference:
                break

        # If we have a stop line, we don't need to move it less than the window height
        if stop_line:
            y_shift = 0

        window.move(left + x_shift, top + y_shift)

    if window.point_count < 5 and not stop_line:
        return None

    line_type = LineType.STOP if stop_line else None
    return Line(window.points, window_height, line_type)


def window_search(
    image: np.ndarray, windows: Iterable[Window], window_height: int, stop_line: bool = False
) -> list[Line]:
    """Search for the windows in the image.

    :param image: The filtered image.
    :param windows: The windows to search for.
    :param window_height: The height of the windows.
    :param stop_line: Whether we are searching for a stop line.
    :return: The lines in the image.
    """
    lines = []
    for window in windows:
        line = process_window(image, window, window_height, stop_line)
        if line is not None:
            lines.append(line)

    return lines


def __shift_no_points(window: Window, window_height: int) -> None:
    """Shift the window if there are no points in it.

    This will shift the window in the same direction as previous windows if enough are found.
    If not, it will move the window upward.

    :param window: The window to shift.
    :param window_height: The height of the window.
    """
    if window.point_count >= 3:
        x_shift, y_shift = window.directions.mean(axis=0)
        window.move(window.x - x_shift, window.y - y_shift, False)
    else:
        window.move(window.x, window.y - window_height, False)


def __window_at_bounds(image: np.ndarray, window: Window, window_height: int) -> bool:
    """Check if the window is at the bounds of the image."""
    return any(
        [
            window.y - window_height < 0,
            window.x - window.margin // 3 < 0,
            window.x + window.margin // 3 >= image.shape[1],
        ]
    )
