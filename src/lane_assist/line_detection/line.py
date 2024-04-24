import numpy as np

from enum import IntEnum


class LineType(IntEnum):
    """The type of line."""

    SOLID = 1
    DASHED = 2
    STOP = 3


class Line:
    """Represents a line in an image.

    Attributes
    ----------
        points: The points of the line.
        line_type: The type of line.

    """

    points: np.ndarray
    line_type: LineType

    def __init__(
        self,
        points: np.ndarray,
        window_height: int = None,
        line_type: LineType = None,
        gaps_allowed: int = 2
    ) -> None:
        """Initialize the line.

        :param points: The points of the line
        :param window_height: The height of a window, used to determine if it is a solid or dashed line.
        :param line_type: The type of line. This can be set if it is known, for example, stop lines.
        """
        self.points = points

        if line_type is not None:
            self.line_type = line_type
            return

        if window_height is None:
            raise ValueError("'window_height' or 'line_type' must be provided.")

        intervals = np.diff(points[:, 1])
        if len(np.where(abs(intervals) == window_height)[0]) > gaps_allowed * 1.5:
            self.line_type = LineType.DASHED
        else:
            self.line_type = LineType.SOLID

    def __eq__(self, other: object) -> bool:
        """Check if the lines are equal."""
        if not isinstance(other, Line):
            return False

        return np.array_equal(self.points, other.points) and self.line_type == other.line_type

    def __ne__(self, other: object) -> bool:
        """Check if the lines are not equal."""
        return not self.__eq__(other)

    def __repr__(self) -> str:
        """Get the string representation of the line."""
        match self.line_type:
            case LineType.SOLID:
                type_str = "SOLID"
            case LineType.DASHED:
                type_str = "DASHED"
            case LineType.STOP:
                type_str = "STOP"
            case _:
                type_str = "UNKNOWN"

        return f"Line(points={len(self.points)}, line_type={type_str})"

    def as_definition(self) -> str:
        """Get the line as a definition."""
        return f"Line(np.array({self.points.tolist()}), line_type=LineType.{self.line_type.name})"
