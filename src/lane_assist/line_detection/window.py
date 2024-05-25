import numpy as np

from src.config import config
from src.utils.other import euclidean_distance


class Window:
    """Class to represent a window in the image.

    Attributes
    ----------
        directions: The directions of the window.
        not_found: The amount of times the window was not found.
        shape: The shape of the window (height, width).
        x: The x position of the window.
        y: The y position of the window.

    """

    directions: np.ndarray
    not_found: int = 0
    shape: tuple[int, int]
    x: int
    y: int

    __points: list[tuple[int, int]]
    __original_shape: tuple[int, int]

    def __init__(self, x: int, y: int, shape: tuple[int, int]) -> None:
        """Initialize the window.

        :param x: The x position of the window.
        :param y: The y position of the window.
        :param shape: The shape of the window (height, width).
        """
        self.x = x
        self.y = y
        self.shape = shape

        self.directions = np.zeros((4, 2), dtype=np.int8)
        self.__points = []
        self.__original_shape = shape

    @property
    def last_point(self) -> tuple[int, int]:
        """The last point in the window."""
        return self.__points[-1]

    @property
    def margin(self) -> int:
        """The margin of the window (x-axis)."""
        return self.shape[1] // 2

    @property
    def points(self) -> np.ndarray:
        """The points in the window."""
        return np.array(self.__points)

    @property
    def point_count(self) -> int:
        """The number of points in the window."""
        return len(self.__points)

    def get_borders(self, image_shape: tuple[int, ...]) -> tuple[int, int, int, int]:
        """Get the borders of the window.

        :param image_shape: The shape of the image.
        :return: The borders of the window (top, bottom, left, right).
        """
        top = max(0, min(int(self.y) - self.shape[0], image_shape[0]))
        bottom = max(0, min(int(self.y), image_shape[0]))
        left = max(0, min(int(self.x - self.margin), image_shape[1]))
        right = max(0, min(int(self.x + self.margin), image_shape[1]))

        return top, bottom, left, right

    def move(self, x: int, y: int, found_points: bool = True) -> None:
        """Move the window to a new position.

        :param x: The new x position.
        :param y: The new y position.
        :param found_points: Whether we found points in the window.
        """
        margin = 1 + config["line_detection"]["window_margin_growth"]
        if found_points:
            if self.point_count == 0 or not self.__is_crowded(x, y):
                self.__points.append((x, y))

                if self.point_count > 1:
                    self.directions = np.roll(self.directions, 1, axis=0)
                    self.directions[0] = [x - self.x, y - self.y]

            self.shape = self.__original_shape
            self.not_found = 0
        else:
            self.shape = (self.shape[0], self.shape[1] * margin)
            if self.point_count > 0:
                self.not_found += 1

        self.x = x
        self.y = y

    def __is_crowded(self, x: int, y: int) -> bool:
        """Check if the new position is too close to the last point.

        :param x: The new x position.
        :param y: The new y position.
        :return: Whether the new position is crowded.
        """
        distance = euclidean_distance(self.__points[-1], (x, y))
        min_distance = self.shape[0] * config["line_detection"]["min_window_shift"]

        return distance < min_distance
