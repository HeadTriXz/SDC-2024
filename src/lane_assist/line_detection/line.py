from enum import IntEnum

import numpy as np


class LineType(IntEnum):
    """The type of line."""

    SOLID = 1
    DASHED = 2
    STOP = 3


class Line:
    """A line in the image.

    This class is used to represent a line in the image.
    It contains the points of the line and the type of line.
    """

    points: np.ndarray
    line_type: LineType

    def __init__(
        self, points: np.ndarray, window_height: int = None, line_type: LineType = None, gaps_allowed: int = 2
    ) -> None:
        """Initialize the line.

        Parameters
        ----------
        :param points: the points of the line
        :param window_height: the height of a window. used to determine if it is a solid or dashed line
        :param line_type: the type of line. this can be set if it is known, for example stop lines.

        """
        self.points = points

        if line_type is not None:
            self.line_type = line_type
            return

        if window_height is None:
            raise ValueError("window_height or line_type must be provided")

        # check if there are regular intervals greater then the height of a window
        # if so we have a dashed line. if there are no regular intervals we have a solid line
        if len(points) < 2:
            self.line_type = LineType.SOLID
            return

        intervals = np.diff(points[:, 1])
        if len(np.where(abs(intervals) > window_height * 2)[0]) > gaps_allowed:
            self.line_type = LineType.DASHED
        else:
            self.line_type = LineType.SOLID

    def __eq__(self, other: object) -> bool:  # noqa: N807, ANN001
        """Check if the lines are equal."""
        if not isinstance(other, Line):
            return False
        return np.array_equal(self.points, other.points) and self.line_type == other.line_type

    def __ne__(self, other: object) -> bool:  # noqa: N807, ANN001
        """Check if the lines are not equal."""
        return not self.__eq__(other)

    def __repr__(self) -> str:
        """Get the string representation of the line."""
        type_str = (
            "SOLID" if self.line_type == LineType.SOLID else "DASHED" if self.line_type == LineType.DASHED else "STOP"
        )
        return f"Line(points={len(self.points)}, line_type={type_str})"
