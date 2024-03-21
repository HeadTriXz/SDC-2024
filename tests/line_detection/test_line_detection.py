import os
import unittest

import cv2
import numpy as np
import pytest
from src.lane_assist.line_detection import Line, get_lines
from tests.line_detection.lines import CORNER, CROSSING, STOP_LINE, STRAIGHT

from lane_assist.image_manipulation.top_down_transfrom import topdown

BENCHMARK_ITERATIONS = 10000


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    return cv2.imread(get_path(file), cv2.IMREAD_GRAYSCALE)


class TestDetectLines(unittest.TestCase):
    """Tests for the line detection methods."""

    def test_line_detection_corner(self) -> None:
        """Test the line detection on the corner image."""
        image = get_image("./images/corner.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, CORNER)

    def test_line_detection_straight(self) -> None:
        """Test the line detection on the straight image."""
        image = get_image("./images/straight.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, STRAIGHT)

    def test_line_detection_crossing(self) -> None:
        """Test the line detection on the crossing image."""
        image = get_image("./images/crossing.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, CROSSING)

    def test_line_detection_stopline(self) -> None:
        """Test the line detection on the stopline image."""
        image = get_image("./images/stopline.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, STOP_LINE)

    def __test_line_detection(self, image: np.ndarray, expected_lines: list[Line]) -> None:
        """Test the line detection on the given image."""
        lines = get_lines(topdown(image))
        with self.subTest("amount of lines detected"):
            self.assertEqual(len(lines), len(expected_lines), "Number of lines is not equal")

        with self.subTest("lines"):
            for i, line in enumerate(lines):
                with self.subTest(f"line {i}"):
                    self.assertEqual(line.line_type, expected_lines[i].line_type, f"Line type of line {i} is not equal")
                    self.assertEqual(
                        len(line.points),
                        len(expected_lines[i].points),
                        f"Number of points in line {i} is not equal",
                    )
                    self.assertEqual(line, expected_lines[i], f"Line {i} is not equal to expected line")

    # run with cprofile when the argument --cprofile is given


@pytest.mark.benchmark(group="line_detection", min_rounds=5, disable_gc=True)
@pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmark if not set")
def test_line_detection_benchmark(benchmark: any) -> None:
    """Benchmark the line detection.

    to make sure it is a realistic situation all images in the test folder are used.
    this benchmark will not include stitching the images.
    """
    benchmark_images = [get_image(f"./images/{img}") for img in os.listdir(get_path("./images"))]
    td_images = [topdown(img) for img in benchmark_images]

    rndi = RandomImage(td_images)

    benchmark(get_lines, rndi.image)


class RandomImage:
    """Class that provides an image from a list of images.

    This class is used to provide an image from a list of images.
    This is due to not being able to iterate over the images normally in the benchmark.
    """

    __images: list[np.ndarray]

    def __init__(self, images: list[np.ndarray]) -> None:
        """Initialize the RandomImage."""
        self.__images = images
        self.__index = 0

    @property
    def image(self) -> np.ndarray:
        """Get the image."""
        self.__index += 1
        return self.__images[self.__index % len(self.__images)]


if __name__ == "__main__":
    pytest.main()
