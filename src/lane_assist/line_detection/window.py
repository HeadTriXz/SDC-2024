import numpy as np


class Window:
    """Class to represent a window in the image."""

    def __init__(self, x: int, y: int, margin: int) -> None:
        """Initialize the window."""
        self.x = x
        self.y = y
        self.margin = margin

        self.collided = False
        self.found_in_previous = False
        self.points = np.zeros((0, 2), dtype=int)

    def move(self, x: int, y: int, points: bool = True) -> None:
        """Move the window to a new position."""
        self.x = x
        self.y = y

        if points:
            self.points = np.vstack((self.points, [[x, y]]))
            self.found_in_previous = True

    def __eq__(self, other: object) -> bool:
        """Check if the windows are equal."""
        if not isinstance(other, Window):
            return NotImplemented
        return self.x == other.x and self.y == other.y
