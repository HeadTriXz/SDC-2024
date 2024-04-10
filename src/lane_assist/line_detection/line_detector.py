import cv2
import numpy as np

from config import config
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window_search import window_search
from typing import Any


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

    return lines[j:i + 1]


def get_lines(image: np.ndarray) -> tuple[list[Line], list[Any]]:
    """Get the lines in the image.

    :param image: The image to get the lines from.
    :return: The lines in the image.
    """
    cv2.threshold(image, config.image_manipulation.white_threshold, 255, cv2.THRESH_BINARY, image)

    return window_search(image, 110)
