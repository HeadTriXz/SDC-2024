import os

import cv2
import numpy as np
import pytest

import config
from lane_assist.lane_assist import PathFollower, filter_lines, generate_driving_path, get_lines
from lane_assist.preprocessing.birdview import topdown
from lane_assist.preprocessing.stitching import stitch_images


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    return cv2.imread(get_path(file))


@pytest.mark.benchmark(group="line_detection", min_rounds=5, disable_gc=True)
@pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmark if not set")
def test_lane_assist_benchmark(benchmark: any) -> None:
    """Benchmark the line detection.

    To make sure it is a realistic situation, all images in the test folder are used.
    This benchmark will not include stitching the images.
    """
    center_img = get_image("../../resources/images/crossing/center.jpg")
    left_img = get_image("../../resources/images/crossing/left.jpg")
    right_img = get_image("../../resources/images/crossing/right.jpg")

    assert center_img is not None
    assert left_img is not None
    assert right_img is not None

    line_following = PathFollower(0, 0, 0)

    def get_steering_angle() -> float:
        """Get the steering angle."""
        nonlocal center_img, left_img, right_img

        # convert to grayscale
        left = cv2.cvtColor(left_img, cv2.COLOR_RGB2GRAY)
        right = cv2.cvtColor(right_img, cv2.COLOR_RGB2GRAY)
        center = cv2.cvtColor(center_img, cv2.COLOR_RGB2GRAY)

        # stitch the images together
        stitched_image = stitch_images(left, center, right)
        topdown_image = topdown(stitched_image)

        # get the lines
        lines = get_lines(topdown_image)
        lines = filter_lines(lines, topdown_image.shape[1] // 2)

        path = generate_driving_path(lines, config.requested_lane)
        return line_following.get_steering_fraction(path, topdown_image.shape[1] // 2)

    # start the benchmark
    benchmark(get_steering_angle)
