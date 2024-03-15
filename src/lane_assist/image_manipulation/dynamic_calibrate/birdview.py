import time

import cv2
import numpy as np

from lane_assist.image_manipulation.dynamic_calibrate.calibrate import calibrate_cameras
from lane_assist.image_manipulation.dynamic_calibrate.utils import adjust_perspective, scale_matrix


def warp_image(image: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Warp an image using a perspective matrix.

    :param image: The image to warp.
    :param matrix: The perspective matrix.
    :return: The warped image.
    """
    matrix, width, height = adjust_perspective(matrix, (image.shape[0], image.shape[1]))

    return cv2.warpPerspective(image, matrix, (width, height))


if __name__ == "__main__":
    image1 = cv2.imread("images/fake1.png")
    image2 = cv2.imread("images/fake2.png")

    image1 = cv2.resize(image1, (1280, 720))[300:]
    image2 = cv2.resize(image2, (1280, 720))[300:]

    images = np.array([image1, image2])
    matrices = calibrate_cameras(images)

    start = time.perf_counter()
    for _ in range(1000):
        for img, matrix in zip(images, matrices):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            result = warp_image(img, matrix)
    end = time.perf_counter()

    print(f"Time: {end - start:.4f} seconds")
    print(f"FPS: {1000 / (end - start):.2f}")
