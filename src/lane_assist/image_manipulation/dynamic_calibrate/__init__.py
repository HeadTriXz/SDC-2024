import cv2
import numpy as np

from lane_assist.image_manipulation.dynamic_calibrate.birdview import warp_image
from lane_assist.image_manipulation.dynamic_calibrate.calibrate import CameraCalibrator


def preprocess(image: np.ndarray) -> np.ndarray:
    """Preprocess an image.

    :param image: The image to preprocess.
    :return: The preprocessed image.
    """
    image = cv2.resize(image, (1280, 720))[300:]
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def calibrate_and_save_params() -> None:
    image1 = cv2.imread("images/street-left.jpg")
    image2 = cv2.imread("images/fake1.png")
    image3 = cv2.imread("images/street-right.jpg")

    image1 = preprocess(image1)
    image2 = preprocess(image2)
    image3 = preprocess(image3)

    calibrator = CameraCalibrator(image1, image2, image3)
    calibrator.calibrate()
    calibrator.save("data")


def merge_images(base_image: np.ndarray, new_image: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """Merge a new image onto a base image with the given offset.

    :param base_image: The base image onto which the new image will be merged.
    :param new_image: The new image to merge onto the base image.
    :param offset: The offset for the new image.
    :return: The merged image.
    """
    # Calculate dimensions for merging
    new_height, new_width = new_image.shape[:2]
    base_height, base_width = base_image.shape[:2]
    offset_x, offset_y = offset

    # Calculate the region of interest (ROI) for merging
    roi_top = max(offset_y, 0)
    roi_bottom = min(offset_y + new_height, base_height)
    roi_left = max(offset_x, 0)
    roi_right = min(offset_x + new_width, base_width)

    # Calculate the cropped region of the new image
    crop_top = roi_top - offset_y
    crop_bottom = crop_top + (roi_bottom - roi_top)
    crop_left = roi_left - offset_x
    crop_right = crop_left + (roi_right - roi_left)

    # Merge the images
    merged_image = np.copy(base_image)
    merged_image[roi_top:roi_bottom, roi_left:roi_right] = new_image[crop_top:crop_bottom, crop_left:crop_right]

    return merged_image


if __name__ == "__main__":
    calibrate_and_save_params()

    image1 = cv2.imread("images/street-left.jpg")
    image2 = cv2.imread("images/fake1.png")
    image3 = cv2.imread("images/street-right.jpg")

    image1 = preprocess(image1)
    image2 = preprocess(image2)
    image3 = preprocess(image3)

    calibrator = CameraCalibrator.load("data/latest.npz")

    warped_center, _ = warp_image(image2, calibrator.matrices[1])
    warped_left, _ = warp_image(image1, calibrator.matrices[0], warped_center.shape[0])
    warped_right, _ = warp_image(image3, calibrator.matrices[2], warped_center.shape[0])

    final_image = np.zeros(calibrator.shape, dtype=np.uint8)
    final_image = merge_images(final_image, warped_center, calibrator.offsets[1])
    final_image = merge_images(final_image, warped_left, calibrator.offsets[0])
    final_image = merge_images(final_image, warped_right, calibrator.offsets[2])

    ...
