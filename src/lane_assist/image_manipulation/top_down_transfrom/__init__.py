import cv2
import numpy as np

from utils.image import cut_image

pts = np.array([[55, 900], [1841, 253], [2067, 253], [3861, 900]], dtype=np.float32)
ipm_pts = np.array([[780, 450], [800, 1100], [600, 1100], [620, 450]], dtype=np.float32)
ipm_matrix = cv2.getPerspectiveTransform(pts, ipm_pts)


def topdown(image: np.ndarray) -> np.ndarray:
    """Transform stitched image to top-down view."""
    if image is None:
        raise ValueError("Error: Unable to load image")

    ipm = cv2.warpPerspective(image, ipm_matrix, (image.shape[1], image.shape[0]), flags=cv2.INTER_NEAREST)
    ipm = cut_image(ipm, 300, 450, 800, 900)
    ipm = cv2.rotate(ipm, cv2.ROTATE_90_CLOCKWISE)
    return cv2.rotate(ipm, cv2.ROTATE_90_CLOCKWISE)


if __name__ == "__main__":
    # load stitched image
    image = cv2.imread("result.jpg")

    # time the topdown function and print the Iterations per second
    import time

    start = time.time()
    for _ in range(500):
        topdown(image)
    end = time.time()
    fps = 500 / (end - start)

    # convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # time again
    start = time.time()
    for _ in range(500):
        topdown(gray)
    end = time.time()
    fps_gray = 500 / (end - start)
