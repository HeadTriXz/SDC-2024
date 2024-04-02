import os
from typing import Generator

import cv2
import numpy as np
import pytest

from lane_assist.lane_assist import LaneAssist, PathFollower
from lane_assist.preprocessing.birdview import topdown

from .mocks.mock_speed_controller import MockSpeedController


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_image(file: str) -> np.ndarray:
    """Get the image from the given file."""
    return cv2.imread(get_path(file), cv2.IMREAD_GRAYSCALE)


@pytest.mark.benchmark(group="test_line_assist", min_rounds=5, disable_gc=True)
@pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmark if not set")
def test_lane_assist_benchmark(benchmark: any) -> None:
    """Benchmark the line detection.

    This benchmark will test the lane assist loop with an image.
    It will only benchmark the lane assist loop and not the image generation

    :param benchmark: The benchmark fixture.
    """
    img = get_image("../../resources/stitched_images/straight.jpg")
    td = topdown(img)

    mock_speed_controller = MockSpeedController()

    def image_generation() -> Generator[np.ndarray | np.ndarray, None, None]:
        """Generate the image."""
        pass

    path_follower = PathFollower(0, 0, 0)
    lane_assist = LaneAssist(
        image_generation,
        path_follower,
        mock_speed_controller,
        adjust_speed=lambda _: 1,
    )

    # start the benchmark
    benchmark(lane_assist.lane_assist_loop, td)
