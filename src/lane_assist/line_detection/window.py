import numpy as np

from src.config import config


class Window:
    """Class to represent a window in the image.

    Attributes
    ----------
        collided: Whether the window collided with another window.
        directions: The directions of the window.
        found_in_previous: Whether we found points in the previous window.
        margin: The margin of the window.
        not_found: The amount of times the window was not found.
        x: The x position of the window.
        y: The y position of the window.

    """

    directions: np.ndarray
    found_in_previous: bool = False
    margin: int
    not_found: int = 0
    x: int
    y: int

    __points: list[tuple[int, int]]

    def __init__(self, x: int, y: int, margin: int) -> None:
        """Initialize the window.

        :param x: The x position of the window.
        :param y: The y position of the window.
        :param margin: The margin of the window.
        """
        self.x = x
        self.y = y
        self.margin = margin

        self.directions = np.zeros((4, 2), dtype=np.int8)
        self.__points = []

    @property
    def points(self) -> np.ndarray:
        """Get the points in the window."""
        return np.array(self.__points)

    @property
    def point_count(self) -> int:
        """Get the number of points in the window."""
        return len(self.__points)

    def move(self, x: int, y: int, points: bool = True) -> None:
        """Move the window to a new position.

        :param x: The new x position.
        :param y: The new y position.
        :param points: Whether we found points in the window.
        """
        margin = 1 + config.line_detection.window_margin_growth / 100
        if points:
            self.__points.append((x, y))

            self.margin //= margin ** self.not_found
            self.found_in_previous = True
            self.not_found = 0

            if len(self.__points) > 1:
                self.directions = np.roll(self.directions, 1, axis=0)
                self.directions[0] = [self.x - x, self.y - y]
        else:
            self.margin *= margin
            if self.found_in_previous:
                self.not_found += 1

        self.x = x
        self.y = y

