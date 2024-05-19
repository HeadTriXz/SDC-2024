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
    while not window.collided:
        if window.y - window_height < 0:
            break

        if window.x - window.margin // 3 < 0:
            break

        if window.x + window.margin // 3 >= image.shape[1]:
            break

        top = max(0, min(window.y - window_height, image.shape[0]))
        bottom = max(0, min(window.y, image.shape[0]))
        left = max(0, min(window.x - int(window.margin), image.shape[1]))
        right = max(0, min(window.x + int(window.margin), image.shape[1]))

        # We should go up if there are no pixels in the window.
        non_zero = np.argwhere(image[top:bottom, left:right])
        if len(non_zero) < config.line_detection.pixels_in_window:
            window.move(window.x, top, False)
            continue

        # Kill the window if we suddenly change direction.
        if window.not_found >= 3:
            x_diff, y_diff = np.mean(window.directions, axis=0)

            direction = abs(abs(np.arctan2(y_diff, x_diff) * 180 / np.pi) - 90)
            if direction > config.line_detection.thresholds.max_angle_difference:
                window.collided = True
                continue

        y_shift, x_shift = np.mean(non_zero, axis=0).astype(int)
        if stop_line:
            y_shift = 0

        window.move(left + x_shift, top + y_shift)

    points = window.points
    if len(points) < 5 and not stop_line:
        return None

    line_type = LineType.STOP if stop_line else None
    return Line(points, window_height, line_type)


def window_search(
        image: np.ndarray,
        windows: Iterable[Window],
        window_height: int,
        stop_line: bool = False
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
