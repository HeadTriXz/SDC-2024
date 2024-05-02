import numpy as np

from scipy.signal import savgol_filter

from src.lane_assist.line_detection.line import Line
from src.utils.calibration_data import CalibrationData


class Path:
    """Represents a path to follow.

    Attributes
    ----------
        points: The points of the path.
        radius: The radius of the path.

    """

    points: np.ndarray
    radius: float

    __calibration: CalibrationData

    def __init__(self, calibration: CalibrationData, points: np.ndarray) -> None:
        """Initialize the path.

        :param calibration: The calibration data.
        :param points: the points of the path.
        """
        self.points = points
        self.__calibration = calibration
        self.__fit_curve()

    def __fit_curve(self) -> None:
        """Fit a curve to the points."""
        self.__fit = np.polyfit(self.points[:, 1], self.points[:, 0], 2)

        # Fit polynomial curve to points in real-world coordinates
        meters_per_pixel = 1 / self.__calibration.pixels_per_meter
        fit_cr = np.polyfit(self.points[:, 1] * meters_per_pixel, self.points[:, 0] * meters_per_pixel, 2)[:2]
        y_eval = np.max(self.points[:, 1])
        a, b = fit_cr

        # Calculate the radius of curvature
        self.radius = (((1 + (2 * a * y_eval * meters_per_pixel + b) ** 2) ** 1.5) / np.absolute(2 * a))

    def __repr__(self) -> str:
        """Get the representation of the path."""
        return f"Path({self.points}, {self.radius})"

    def __str__(self) -> str:
        """Get the string representation of the path."""
        return f"Path({self.points}, {self.radius})"


def generate_driving_path(calibration: CalibrationData, lines: list[Line], requested_lane: int) -> Path:
    """Generate the driving path based on the lines.

    :param calibration: The calibration data.
    :param requested_lane: The lane we want to drive in.
    :param lines: The lines to generate the path from.
    :return: The generated path.
    """
    lanes = [[lines[i], lines[i - 1]] for i in range(len(lines) - 1, 0, -1)]

    if requested_lane >= len(lanes):
        requested_lane = len(lanes) - 1

    # Get the lines of the lane we want to drive in.
    a1 = lanes[requested_lane][0]
    a2 = lanes[requested_lane][1]

    # Interpolate the lines to get the same amount of points.
    inter_line_points = max(100, min(len(a1.points), len(a2.points)) * 4)

    new_a1_x, new_a1_y = interpolate_line(a1, inter_line_points)
    new_a2_x, new_a2_y = interpolate_line(a2, inter_line_points)

    # Generate a centerline based on the two lines.
    midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(inter_line_points)]
    midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(inter_line_points)]

    # Smooth the centerline.
    midx = savgol_filter(midx, 51, 3)
    midy = savgol_filter(midy, 51, 3)

    return Path(calibration, np.array([midx, midy]).T)


def interpolate_line(line: Line, points: int) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate the line to the given points.

    This function will interpolate the line to the given points.

    :param line: The line to interpolate.
    :param points: The points to interpolate to.
    :return: The interpolated line.
    """
    new_y = np.linspace(line.points[0, 1], line.points[-1, 1], points)
    new_x = np.interp(new_y, line.points[::-1, 1], line.points[::-1, 0])

    return new_x, new_y
