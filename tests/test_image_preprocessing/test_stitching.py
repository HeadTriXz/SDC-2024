import os

import cv2
import numpy as np
import pytest

from lane_assist.preprocessing.stitching import stitch_images


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    filepath = get_path(file)
    if not os.path.exists(filepath):
        print(f"File {filepath} not found")
    return cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)


@pytest.mark.benchmark(group="test_line_assist", min_rounds=5, disable_gc=False)
# @pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmark if not set")
def test_lane_assist_benchmark(benchmark: any) -> None:
    """Benchmark the line detection.

    This benchmark will test the lane assist loop with an image.
    It will only benchmark the lane assist loop and not the image generation

    :param benchmark: The benchmark fixture.
    """
    center = get_image("../../resources/unstitched_images/straight/center.jpg")
    left = get_image("../../resources/unstitched_images/straight/left.jpg")
    right = get_image("../../resources/unstitched_images/straight/right.jpg")

    # resie images to 720*1280
    center = cv2.resize(center, (1280, 720))
    left = cv2.resize(left, (1280, 720))
    right = cv2.resize(right, (1280, 720))

    print(center.shape)

    benchmark(stitch_images, center, left, right)
