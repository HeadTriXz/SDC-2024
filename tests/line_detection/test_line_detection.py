import os
import time
import unittest

import cv2
import numpy as np
from src.lane_assist.line_detection import Line, get_lines

from .lines import CORNER, CROSSING, STOP_LINE, STRAIGHT

BENCHMARK_ITERATIONS = 500


class TestDetectLines(unittest.TestCase):
    """Tests for the line detection methods."""

    def test_line_detection_corner(self) -> None:
        """Test the line detection on the corner image."""
        image = cv2.imread("./line_detection/images/corner.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, CORNER)

    def test_line_detection_straight(self) -> None:
        """Test the line detection on the straight image."""
        image = cv2.imread("./line_detection/images/straight.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, STRAIGHT)

    def test_line_detection_crossing(self) -> None:
        """Test the line detection on the crossing image."""
        image = cv2.imread("./line_detection/images/crossing.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, CROSSING)

    def test_line_detection_stopline(self) -> None:
        """Test the line detection on the stopline image."""
        image = cv2.imread("./line_detection/images/stopline.jpg")
        self.assertIsNotNone(image, "Image not found")
        self.__test_line_detection(image, STOP_LINE)

    def __test_line_detection(self, image: np.ndarray, expected_lines: list[Line]) -> None:
        """Test the line detection on the given image."""
        lines = get_lines(image)
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

    @unittest.skipIf(os.environ.get("SKIP_BENCHMARK", "0") == "1", "benchmarking not enabled")
    def test_benchmark(self) -> None:
        """Benchmark the line detection.

        to make sure it is a realistic situation all images in the test folder are used.
        this benchmark will not include stitching the images.
        """
        benchmark_images = [
            cv2.imread(f"./line_detection/images/{img}") for img in os.listdir("./line_detection/images")
        ]

        img_count = len(benchmark_images)

        start = time.process_time()
        for i in range(BENCHMARK_ITERATIONS):
            get_lines(benchmark_images[i % img_count])
        stop = time.process_time()
        fps = BENCHMARK_ITERATIONS / (stop - start)
        per_iteration = (stop - start) / BENCHMARK_ITERATIONS

        print(f"Line detection: FPS: {fps:.4f}, per iteration: {per_iteration:.6f}")  # noqa: T201


if __name__ == "__main__":
    unittest.main()
