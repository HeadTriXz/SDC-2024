import cv2
import numpy as np

from lane_assist.preprocessing.birdview import warp_image
from lane_assist.preprocessing.calibrate import CameraCalibrator
from lane_assist.preprocessing.stitching import stitch_images


def get_stitched_image(calibrator: CameraCalibrator, images: list[np.ndarray]) -> np.ndarray:
    """Get the stitched image.

    :param calibrator: The camera calibrator.
    :param images: The images to stitch.
    :return: The stitched image.
    """
    images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]

    warped_center = warp_image(images[1], calibrator.matrices[1])
    warped_left = warp_image(images[0], calibrator.matrices[0])
    warped_right = warp_image(images[2], calibrator.matrices[2])

    stitched = np.zeros(calibrator.output_shape, dtype=np.uint8)
    stitched = stitch_images(stitched, warped_right, calibrator.offsets[2])
    stitched = stitch_images(stitched, warped_left, calibrator.offsets[0])
    return stitch_images(stitched, warped_center, calibrator.offsets[1])


def main() -> None:
    filenames = ["left.jpg", "center.jpg", "right.jpg"]
    calib_images = [cv2.imread(f"images/rotated/{img}") for img in filenames]
    calib_images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in calib_images]

    calibrator = CameraCalibrator(calib_images)
    calibrator.calibrate()

    test_names = ["left.png", "center.png", "right.png"]
    test_images = [cv2.imread(f"images/test/{img}") for img in test_names]

    stitched_image = get_stitched_image(calibrator, test_images)
    ...


if __name__ == "__main__":
    main()
