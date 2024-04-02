import cv2
import numpy as np

SRC_PTS = np.array([[55, 900], [1841, 253], [2067, 253], [3861, 900]], dtype=np.float32)
DST_PTS = np.array([[780, 450], [800, 1100], [600, 1100], [620, 450]], dtype=np.float32)
MATRIX = cv2.getPerspectiveTransform(SRC_PTS, DST_PTS)


def cut_image(image: np.ndarray, x: int, y: int, width: int, height: int) -> np.ndarray:
    """Cut the image to specified width and height."""
    return image[y : y + height, x : x + width]


def topdown(image: np.ndarray) -> np.ndarray:
    """Transform stitched image to top-down view."""
    img = cv2.warpPerspective(image, MATRIX, (1200, 1250), flags=cv2.INTER_NEAREST)
    img = cut_image(img, 300, 450, 800, 900)
    return cv2.rotate(img, cv2.ROTATE_180)
