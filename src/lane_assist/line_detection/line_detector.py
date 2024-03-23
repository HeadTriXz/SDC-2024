import cv2
import numpy as np

from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window_search import window_search


def filter_lines(lines: list[Line], starting_point: int, ret_stoplines: bool = False) -> list[Line]:
    """Get the lines between the solid lines closest to each side of the starting point."""
    solid_lines = [line for line in lines if line.line_type != LineType.STOP]
    i = 0
    j = 0
    while i < len(solid_lines):
        # check if we are after the starting point
        if solid_lines[i].points[0][0] >= starting_point and solid_lines[i].line_type == LineType.SOLID:
            # back up until we find a solid line
            j = i
            while j > 0:
                j -= 1
                if lines[j].line_type == LineType.SOLID:
                    break
            break
        i += 1

    if ret_stoplines:
        return solid_lines[j: i + 1] + [line for line in lines if line.line_type == LineType.STOP]

    return solid_lines[j: i + 1]


def get_lines(image: np.ndarray) -> list[Line]:
    """Get the lines in the image.

    This function will take an image and return the lines in the image.
    the image should be stitched and not top down
    """
    white = cv2.inRange(image, 200, 255)
    return window_search(white, 110)
