import math
import numpy as np

from scipy.signal import savgol_filter

from src.calibration.data import CalibrationData
from src.config import config
from src.lane_assist.line_detection.line import Line
from src.utils.other import euclidean_distance


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
        meters_per_pixel = self.__calibration.get_distance(1)

        # Fit a second-degree polynomial to the points.
        coefficients = np.polyfit(self.points[:, 1] * meters_per_pixel, self.points[:, 0] * meters_per_pixel, 2)

        # Get the maximum y-coordinate of the points.
        y_eval = np.max(self.points[:, 1])
        a, b = coefficients[:2]

        # Calculate the radius of curvature
        self.radius = ((1 + (2 * a * y_eval * meters_per_pixel + b) ** 2) ** 1.5) / np.absolute(2 * a)

    def __repr__(self) -> str:
        """Get the representation of the path."""
        return f"Path({self.points}, {self.radius})"

    def __str__(self) -> str:
        """Get the string representation of the path."""
        return f"Path({self.points}, {self.radius})"


def compute_normals(line: Line) -> np.ndarray:
    """Compute the normals of the line.

    :param line: The line to compute the normals for.
    :return: The normals of the line.
    """
    dx = np.gradient(line.points[:, 0])
    dy = np.gradient(line.points[:, 1])

    # Compute the normals.
    normals = np.array([-dy, dx]).T

    # Normalize the normals.
    normals /= np.linalg.norm(normals, axis=1)[:, np.newaxis]
    return normals


def offset_line(line: Line, offset: float) -> np.ndarray:
    """Offset the line by the given offset.

    :param line: The line to offset.
    :param offset: The offset to apply.
    :return: The offset line.
    """
    normals = compute_normals(line)
    return line.points + offset * normals


def generate_driving_path(
        calibration: CalibrationData,
        lines: list[Line],
        requested_lane: int,
        current_position: tuple[int, int]
) -> Path:
    """Generate the driving path based on the lines.

    :param calibration: The calibration data.
    :param requested_lane: The lane we want to drive in.
    :param lines: The lines to generate the path from.
    :param current_position: The current position of the go-kart.
    :return: The generated path.
    """
    if len(lines) == 1:
        offset = current_position[0] - lines[0].points[0, 0]
        path = offset_line(lines[0], offset)

        return Path(calibration, path)

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
    center_x = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(inter_line_points)]
    center_y = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(inter_line_points)]

    # Smooth the centerline.
    center_x = savgol_filter(center_x, 51, 3)
    center_y = savgol_filter(center_y, 51, 3)

    # Find the intersection point.
    dist = calibration.get_distance(abs(center_x[0] - current_position[0]))
    if dist > config["line_following"]["look_ahead_distance"] * 2:
        look_ahead_padding = max(0.0, math.log(dist + 1) * 2)
        look_ahead_px = calibration.get_pixels(dist + look_ahead_padding)

        i = 0
        for i in range(len(center_x)):
            distance = euclidean_distance(current_position, (center_x[i], center_y[i]))
            if distance > look_ahead_px:
                break

        intersection_point = np.array([center_x[i], center_y[i]])
        feeler_points = np.concatenate([np.linspace(current_position, intersection_point, look_ahead_px // 3)])

        # Add the feeler points to the centerline.
        center_x = np.concatenate([feeler_points[:, 0], center_x[i:]])
        center_y = np.concatenate([feeler_points[:, 1], center_y[i:]])

    return Path(calibration, np.array([center_x, center_y]).T)


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
