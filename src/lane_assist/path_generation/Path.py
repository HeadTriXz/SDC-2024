import numpy as np


class Path:
    """The path class.

    This class is used to represent a path. this is a more complex version of a line.
    this will also contain the minimum radius of the curvature of the path.
    """

    def __init__(self, points: np.ndarray):
        self.points = points



    def get_path_points(self):
        return self.path_points

    def get_path_type(self):
        return self.path_type
