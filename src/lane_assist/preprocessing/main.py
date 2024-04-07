import cv2
import numpy as np
import time

from lane_assist.preprocessing.birdview import warp_image
from lane_assist.preprocessing.calibrate import CameraCalibrator
from lane_assist.preprocessing.stitching import stitch_images
from lane_assist.preprocessing.utils.other import calculate_output_shape


def get_stitched_image(calibrator: CameraCalibrator, images: list[np.ndarray]) -> np.ndarray:
    """Get the stitched image.

    :param calibrator: The camera calibrator.
    :param images: The images to stitch.
    :return: The stitched image.
    """
    images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]
    images = [cv2.resize(image, (854, 480)) for image in images]

    warped_left = warp_image(calibrator, images[0], idx=0)
    warped_right = warp_image(calibrator, images[2], idx=2)

    width, height = calculate_output_shape(calibrator.offsets, calibrator.shapes)

    stitched = np.zeros((height, width), dtype=np.uint8)
    stitched = stitch_images(stitched, warped_right, calibrator.offsets[2])
    stitched = stitch_images(stitched, warped_left, calibrator.offsets[0])
    stitched = stitch_images(stitched, images[1], calibrator.offsets[1])

    return cv2.warpPerspective(
        stitched,
        calibrator.topdown_matrix,
        calibrator.output_shape,
        flags=cv2.INTER_NEAREST
    )


def main() -> None:
    filenames = ["left.jpg", "center.jpg", "right.jpg"]
    calib_images = [cv2.imread(f"images/rotated/{img}") for img in filenames]
    calib_images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in calib_images]

    calibrator = CameraCalibrator(calib_images, input_shape=(854, 480))
    calibrator.calibrate()

    test_names = ["left.png", "center.png", "right.png"]
    test_images = [cv2.imread(f"images/test/{img}") for img in test_names]
    test_images = [cv2.resize(image, (1280, 720)) for image in test_images]

    start = time.perf_counter()
    it = 5000

    for _ in range(it):
        stitched_image = get_stitched_image(calibrator, test_images)

    end = time.perf_counter()
    diff = end - start

    print(f"Time taken: {diff:.2f} seconds")
    print(f"Average time: {diff / it:.2f} seconds")
    print(f"FPS: {it / diff:.2f}")


if __name__ == "__main__":
    main()
