import cv2
import numpy as np

from collections.abc import Iterable

from src.config import config
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.window import Window


def process_window(image: np.ndarray, window: Window, window_height: int, stop_line: bool, rgb_image) -> Line | None:
    """Process the window.

    :param image: The image to process.
    :param window: The window to process.
    :param window_height: The height of the window.
    :param stop_line: Whether we are searching for a stop line.
    :return: The processed window.
    """
    while True:
        top = max(0, min(window.y - window_height, image.shape[0]))
        bottom = max(0, min(window.y, image.shape[0]))
        left = max(0, min(window.x - int(window.margin), image.shape[1]))
        right = max(0, min(window.x + int(window.margin), image.shape[1]))

        if window.y - window_height < 0:
            # draw a red box if the window is out of bounds
            cv2.rectangle(rgb_image, (left, top), (right, bottom), (0, 0, 255), 1)
            break

        if window.x - window.margin // 3 < 0:
            cv2.rectangle(rgb_image, (left, top), (right, bottom), (0, 0, 255), 1)
            break

        if window.x + window.margin // 3 >= image.shape[1]:
            cv2.rectangle(rgb_image, (left, top), (right, bottom), (0, 0, 255), 1)
            break

        # We should go up if there are no pixels in the window.
        non_zero = np.argwhere(image[top:bottom, left:right])
        if len(non_zero) < config.line_detection.pixels_in_window:
            # get the difference of the last 3 points in each axis
            if len(window.points) >= 3:
                # get the latest move of the window
                x_diff, y_diff = np.diff(window.points, axis=0).mean(axis=0)
                x_shift = int(x_diff)
                y_shift = int(y_diff)

                cv2.line(rgb_image, (window.x, window.y), (window.x + x_shift, window.y + y_shift), (255, 0, 255), 1)
                window.move(window.x + x_shift, window.y + y_shift, False)

            else:
                window.move(window.x, top, False)

            continue

        y_shift, x_shift = np.mean(non_zero, axis=0).astype(int)

        # check if we move at least 2 px
        if np.linalg.norm([x_shift, y_shift]) < 5 and len(window.points) >= 2:
            try:
                diffs = np.diff(window.points[-2:], axis=0)
                x_diff, y_diff = diffs[:, 0], diffs[:, 1]

                x_diff = x_diff.mean()
                y_diff = y_diff.mean()

                x_shift = int(x_diff)
                y_shift = int(y_diff)

                cv2.line(rgb_image, (window.x, window.y), (window.x + x_shift, window.y + y_shift), (255, 0, 255), 1)
                window.move(window.x + x_shift, window.y + y_shift, False)
                continue

            except Exception as e:
                pass

        # Kill the window if we suddenly change direction.
        if window.not_found >= 3 and len(window.points) >= 3:
            # get the angle from the last known point to the current point
            last_point = window.points[-2]
            current_point = [window.x, window.y]
            x_diff, y_diff = current_point - last_point
            current_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi - 90)

            # get the angle of the line
            prev_point = window.points[-3]
            x_diff, y_diff = np.diff([prev_point, last_point], axis=0).mean(axis=0)
            prev_direction = abs(np.arctan2(y_diff, x_diff) * 180 / np.pi - 90)

            angle_diff = abs(prev_direction - current_direction)
            if angle_diff > config.line_detection.thresholds.max_angle_difference:
                # draw pink box if the window is killed
                cv2.rectangle(rgb_image, (left, top), (right, bottom), (255, 0, 255), 1)
                # write the angle difference
                cv2.putText(
                    rgb_image, f"{angle_diff:.2f}", (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1
                )
                break


        if stop_line:
            y_shift = 0

        # draw a blue line segment if we found pixels in the window
        cv2.line(rgb_image, (window.x, window.y), (left + x_shift, top + y_shift), (255, 0, 0), 1)

        window.move(left + x_shift, top + y_shift)

    points = window.points
    if len(points) < 5 and not stop_line:
        return None

    line_type = LineType.STOP if stop_line else None
    return Line(points, window_height, line_type)


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
    rgb_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

    lines = []
    for window in windows:
        line = process_window(image, window, window_height, stop_line, rgb_image)
        if line is not None:
            lines.append(line)

    # save the image with the windows drawn on it
    cv2.imshow("stop" if stop_line else "driving", rgb_image)
    cv2.waitKey(1)

    return lines
