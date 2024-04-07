import os

import cv2
import numpy as np
import pytest

from lane_assist.preprocessing.birdview import topdown


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    return cv2.imread(get_path(file), cv2.IMREAD_GRAYSCALE)


@pytest.mark.benchmark(group="test_line_assist", min_rounds=5, disable_gc=True)
# @pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmark if not set")
def test_lane_assist_benchmark(benchmark: any) -> None:
    """Benchmark the line detection.

    This benchmark will test the lane assist loop with an image.
    It will only benchmark the lane assist loop and not the image generation

    :param benchmark: The benchmark fixture.
    """
    img = get_image("../../resources/stitched_images/straight.jpg")
    benchmark(topdown, img)
