import numpy as np


class Window:
    """Class to represent a window in the image.

    Attributes
    ----------
        collided: Whether the window collided with another window.
        direction_changes: The direction changes in the window.
        found_in_previous: Whether we found points in the previous window.
        margin: The margin of the window.
        point_index: The index of the point.
        x: The x position of the window.
        y: The y position of the window.

    """

    collided: bool
    found_in_previous: bool
    margin: int
    point_index: int
    x: int
    y: int

    __points: np.ndarray

    def __init__(self, x: int, y: int, margin: int, max_point_count: int = 120) -> None:
        """Initialize the window.

        :param x: The x position of the window.
        :param y: The y position of the window.
        :param margin: The margin of the window.
        :param max_point_count: The maximum amount of points the window can hold.
        """
        self.x = x
        self.y = y
        self.margin = margin

        self.collided = False
        self.found_in_previous = False
        self.point_index = 0

        self.__points = np.zeros((max_point_count, 2), dtype=int)

    @property
    def point_count(self) -> int:
        """Get the number of points in the window."""
        return self.point_index

    @property
    def points(self) -> np.ndarray:
        """Get the points in the window."""
        return self.__points[:self.point_count]  # Use point_count for clarity

    def move(self, x: int, y: int, points: bool = True) -> None:
        """Move the window to a new position.

        :param x: The new x position.
        :param y: The new y position.
        :param points: Whether we found points in the window.
        """
        self.x = x
        self.y = y

        if points:
            self.found_in_previous = True
            self.__points[self.point_index] = [x, y]
            self.point_index += 1
        else:
            self.margin += 0.75

    def collides(self, other: "Window") -> bool:
        """Check if the window collides with another window.

        :param other: The other window.
        :return: Whether the window collides with the other window.
        """
        return (self.x - self.margin < other.x + other.margin and
                self.x + self.margin > other.x - other.margin)

    def __eq__(self, other: object) -> bool:
        """Check if the windows are equal."""
        if not isinstance(other, Window):
            return NotImplemented

        return self.x == other.x and self.y == other.y
