import cv2
import os
import numpy as np
import pytest

from pytest_benchmark.fixture import BenchmarkFixture
from src.lane_assist.image_manipulation.dynamic_calibrate.birdview import warp_image
from src.lane_assist.image_manipulation.dynamic_calibrate.calibrate import CameraCalibrator
from src.lane_assist.image_manipulation.dynamic_calibrate.stitch import stitch_images

BENCHMARK_ITERATIONS = 100
ROI_CROP_TOP = 200


def get_path(file: str) -> str:
    """Get the absolute path of the current file."""
    current_file = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(current_file, file))


def get_stitched_image(calibrator: CameraCalibrator, images: list[np.ndarray]) -> np.ndarray:
    """Get the stitched image.

    :param calibrator: The camera calibrator.
    :param images: The images to stitch.
    :return: The stitched image.
    """
    images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]

    warped_center = warp_image(images[1], calibrator.matrices[1])
    warped_left = warp_image(images[0], calibrator.matrices[0], warped_center.shape[0])
    warped_right = warp_image(images[2], calibrator.matrices[2], warped_center.shape[0])

    stitched = np.zeros(calibrator.shape, dtype=np.uint8)
    stitched = stitch_images(stitched, warped_left, calibrator.offsets[0])
    stitched = stitch_images(stitched, warped_right, calibrator.offsets[2])
    stitched = stitch_images(stitched, warped_center, calibrator.offsets[1])

    return stitched


@pytest.mark.benchmark(group="dynamic_calibration", min_rounds=5)
@pytest.mark.skipif("BENCHMARK" not in os.environ, reason="Skip benchmarks if not explicitly requested")
def test_dynamic_calibration_benchmark(benchmark: BenchmarkFixture) -> None:
    filenames = ["left.png", "center.png", "right.png"]
    images = [cv2.imread(get_path(f"images/{img}")) for img in filenames]
    images = [cv2.resize(img, (1280, 720))[ROI_CROP_TOP:] for img in images]

    calibrator = CameraCalibrator(images)
    calibrator.calibrate()

    benchmark(get_stitched_image, calibrator, images)


if __name__ == "__main__":
    pytest.main()
