import cv2
import numpy as np
import numpy.testing as npt
import os
import unittest

from src.lane_assist.line_detection.line import Line
from src.lane_assist.line_detection.line_detector import filter_lines, get_lines
from src.lane_assist.line_following.path_generator import generate_driving_path
from src.lane_assist.preprocessing.birdview import topdown

from .test_data import CORNER, CROSSING_LANE_0, CROSSING_LANE_1, STOPLINE, STRAIGHT

BENCHMARK_ITERATIONS = 10000


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    return cv2.imread(get_path(file), cv2.IMREAD_GRAYSCALE)


class TestGenerateDrivingLine(unittest.TestCase):
    """Tests for the line detection methods."""

    def __get_lines(self, img: str) -> list[Line]:
        img = get_image(img)
        td_img = topdown(img)

        return get_lines(td_img)

    def test_line_generation_corner(self) -> None:
        """Test the line detection on the corner image."""
        base_lines = self.__get_lines("../line_detection/images/corner.jpg")
        filtered = filter_lines(base_lines, 400)
        self.assertEqual(len(filtered), 2)

        driving_path = generate_driving_path(filtered, 0)
        npt.assert_almost_equal(driving_path, CORNER)

    def test_line_generation_straight(self) -> None:
        """Test the line detection on the straight image."""
        base_lines = self.__get_lines("../line_detection/images/straight.jpg")
        filtered = filter_lines(base_lines, 400)
        self.assertEqual(len(filtered), 2)

        driving_path = generate_driving_path(filtered, 0)
        npt.assert_almost_equal(driving_path, STRAIGHT)

    def test_line_generation_crossing(self) -> None:
        """Test the line detection on the crossing image."""
        base_lines = self.__get_lines("../line_detection/images/crossing.jpg")
        filtered = filter_lines(base_lines, 400)
        self.assertEqual(len(filtered), 3)

        with self.subTest("first lane"):
            driving_path = generate_driving_path(filtered, 0)
            npt.assert_almost_equal(driving_path, CROSSING_LANE_0)

        with self.subTest("second lane"):
            driving_path = generate_driving_path(filtered, 1)
            npt.assert_almost_equal(driving_path, CROSSING_LANE_1)

    def test_line_generation_stopline(self) -> None:
        """Test the line detection on the stopline image."""
        base_lines = self.__get_lines("../line_detection/images/stopline.jpg")
        filtered = filter_lines(base_lines, 400)

        driving_path = generate_driving_path(filtered, 0)
        npt.assert_almost_equal(driving_path, STOPLINE)


if __name__ == "__main__":
    unittest.main()
