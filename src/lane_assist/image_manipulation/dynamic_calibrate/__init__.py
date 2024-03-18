import cv2
import numpy as np

from lane_assist.image_manipulation.dynamic_calibrate.birdview import warp_image
from lane_assist.image_manipulation.dynamic_calibrate.calibrate import CameraCalibrator


def preprocess(image: np.ndarray) -> np.ndarray:
    """Preprocess an image.

    :param image: The image to preprocess.
    :return: The preprocessed image.
    """
    image = cv2.resize(image, (1280, 720))[200:]
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def calibrate_and_save_params() -> None:
    image1 = cv2.imread("images/street-left.jpg")
    image2 = cv2.imread("images/street-center.jpg")
    image3 = cv2.imread("images/street-right.jpg")

    image1 = preprocess(image1)
    image2 = preprocess(image2)
    image3 = preprocess(image3)

    calibrator = CameraCalibrator(image1, image2, image3)
    calibrator.calibrate()
    calibrator.find_offsets()
    calibrator.save("data")


if __name__ == "__main__":
    calibrate_and_save_params()

    image1 = cv2.imread("images/street-left.jpg")
    image2 = cv2.imread("images/street-center.jpg")
    image3 = cv2.imread("images/street-right.jpg")

    image1 = preprocess(image1)
    image2 = preprocess(image2)
    image3 = preprocess(image3)

    calibrator = CameraCalibrator()
    calibrator.load("data/latest.npz")

    warped_center, _ = warp_image(image2, calibrator.matrices[1])
    warped_left, _ = warp_image(image1, calibrator.matrices[0], warped_center.shape[0])
    warped_right, _ = warp_image(image3, calibrator.matrices[2], warped_center.shape[0])

    final_image = np.zeros(calibrator.shape, dtype=np.uint8)
    final_image[calibrator.offsets[0, 1]:warped_left.shape[0] + calibrator.offsets[0, 1], calibrator.offsets[0, 0]:warped_left.shape[1] + calibrator.offsets[0, 0]] = warped_left
    # final_image[calibrator.offsets[2, 1]:warped_right.shape[0] + calibrator.offsets[2, 1], calibrator.offsets[2, 0]:warped_right.shape[1] + calibrator.offsets[2, 0]] = warped_right
    final_image[0:warped_center.shape[0], 0:warped_center.shape[1]][warped_center.nonzero()] = warped_center[warped_center.nonzero()]
    ...
