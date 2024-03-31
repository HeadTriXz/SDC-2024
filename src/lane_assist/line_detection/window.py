import numpy as np


class Window:
    """Class to represent a window in the image."""

    def __init__(self, x: int, y: int, margin: int, max_point_count: int = 120) -> None:
        """Initialize the window."""
        self.direction_changes = []
        self.x = x
        self.y = y
        self.margin = margin

        self.collided = False
        self.found_in_previous = False
        self.__points = np.zeros((0, 2), dtype=int)
        self.__new_points = np.zeros((max_point_count, 2), dtype=int)

        self.__orig_margin = margin
        self.__non_found = 0
        self.point_index = 0

    def move(self, x: int, y: int, points: bool = True) -> None:
        """Move the window to a new position."""
        self.x = x
        self.y = y

        if points:
            self.found_in_previous = True
            self.margin = self.__orig_margin
            self.__non_found = 0
            self.__new_points[self.point_index] = [x, y]
            self.point_index += 1
        else:
            self.margin += 0.75
            self.__non_found += 1

    @property
    def point_count(self) -> int:
        """Get the number of points in the window."""
        return self.point_index

    @property
    def points(self) -> np.ndarray:
        """Get the points in the window."""
        return self.__new_points[: self.point_index]

    def collides(self, other: "Window") -> bool:
        """Check if the window collides with another window."""
        return self.x - self.margin < other.x + other.margin and self.x + self.margin > other.x - other.margin

    def __eq__(self, other: object) -> bool:
        """Check if the windows are equal."""
        if not isinstance(other, Window):
            return NotImplemented
        return self.x == other.x and self.y == other.y
