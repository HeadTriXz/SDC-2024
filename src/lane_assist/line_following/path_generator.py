import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

from lane_assist.line_detection.line import Line
from lane_assist.line_detection.line_detector import filter_lines, get_lines


class Path:
    """A class to represent a path."""

    points: np.ndarray
    radius: float

    def __init__(self, points: np.ndarray) -> None:
        """Initialize the path."""
        self.points = points

        self.__fit = np.polyfit(points[:, 1], points[:, 0], 2)
        ym_per_pix = 1 / 19.2495
        xm_per_pix = 1 / 19.2495

        fit_cr = np.polyfit(points[:, 1] * ym_per_pix, points[:, 0] * xm_per_pix, 2)
        y_eval = np.max(points[:, 1])
        self.radius = ((1 + (2 * fit_cr[0] * y_eval * ym_per_pix + fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * fit_cr[0])

    def __repr__(self) -> str:
        """Get the representation of the path."""
        return f"Path({self.points}, {self.radius})"

    def __str__(self) -> str:
        """Get the string representation of the path."""
        return f"Path({self.points}, {self.radius})"


def generate_driving_path(lines: list[Line], requested_lane: int) -> Path:
    """Generate the driving path based on the lines.

    This function will take the lines and generate a driving path based on the lines.

    :param requested_lane: the lane we want to drive in
    :param lines: the lines to generate the path from
    :return: the driving path
    """
    lanes = [[lines[i], lines[i - 1]] for i in range(len(lines) - 1, 0, -1)]

    if requested_lane >= len(lanes):
        requested_lane = len(lanes) - 1

    # get the lines of the lane we want to drive in.
    a1 = lanes[requested_lane][0]
    a2 = lanes[requested_lane][1]

    # interpolate the lines so we have the same amount adn enough points.
    inter_line_points = max(100, min(len(a1.points), len(a2.points)) * 4)

    new_a1_x, new_a1_y = interpolate_line(a1, inter_line_points)
    new_a2_x, new_a2_y = interpolate_line(a2, inter_line_points)

    # generate a centerline based on the two lines
    midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(inter_line_points)]
    midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(inter_line_points)]

    # smothen the line using cumsum
    midx = savgol_filter(midx, 51, 3)
    midy = savgol_filter(midy, 51, 3)

    return Path(np.array([midx, midy]).T)


def interpolate_line(line: Line, points: int) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate the line to the given points.

    This function will interpolate the line to the given points.

    :param line: the line to interpolate
    :param points: the points to interpolate to
    :return: the interpolated line
    """
    new_y = np.linspace(line.points[0, 1], line.points[-1, 1], points)
    new_x = np.interp(new_y, line.points[::-1, 1], line.points[::-1, 0])

    return new_x, new_y
