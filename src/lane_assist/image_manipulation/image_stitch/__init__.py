import math

import cv2
import numpy as np

# Constants
MIN_HEIGHT = 0
MIN_WIDTH = 0
MAX_HEIGHT = 720
MAX_WIDTH = 1280

MASK_OFFSET = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
PTS_ORIGINAL = np.float32([[MIN_WIDTH, MIN_HEIGHT], [MIN_WIDTH, MAX_HEIGHT],
                           [MAX_WIDTH, MAX_HEIGHT], [MAX_WIDTH, MIN_HEIGHT]])

RATIOS_LEFT = np.float32([[0, 1.0055555], [0.36197916, 2.6185186], [1.6765625, 0.7537037], [1.5010417, 0]])
RATIOS_RIGHT = np.float32([[0.18125, 0], [0, 0.74907407], [1.28125, 2.55833333], [1.66770833, 0.99722222]])

LEFT_X, LEFT_Y = 0.27027708, 0.5205  # 0.52165085
RIGHT_X, RIGHT_Y = 0.73115027, 0.5145  # 0.51505412
CENTER_X, CENTER_Y = 0.5049538, 0.1826793


# Calculate transformation matrices
def get_points(ratio: np.ndarray) -> np.ndarray:
    """Get the points of the warped image based on the ratio."""
    return np.float32(ratio * [MAX_WIDTH, MAX_HEIGHT])


def get_matrix(ratio: np.ndarray) -> np.ndarray:
    """Get the transformation matrix based on the ratio."""
    return cv2.getPerspectiveTransform(PTS_ORIGINAL, get_points(ratio))


MATRIX_LEFT = get_matrix(RATIOS_LEFT)
MATRIX_RIGHT = get_matrix(RATIOS_RIGHT)

LEFT_WIDTH = np.max(get_points(RATIOS_LEFT)[:, 0]).astype(int)
LEFT_HEIGHT = np.max(get_points(RATIOS_LEFT)[:, 1]).astype(int)
RIGHT_WIDTH = np.max(get_points(RATIOS_RIGHT)[:, 0]).astype(int)
RIGHT_HEIGHT = np.max(get_points(RATIOS_RIGHT)[:, 1]).astype(int)


# Functions
def relative_to_absolute(x: float, y: float, width: float, height: float) -> tuple[float, float]:
    """Convert relative coordinates to absolute coordinates."""
    return x * width, y * height


def get_ltbr(x: int, y: int, width: int, height: int) -> tuple[int, int, int, int]:
    """Get the left, top, right, and bottom coordinates of the image based on xywh."""
    return (int(round(x - (width / 2))),
            int(round(y - (height / 2))),
            int(round(x + (width / 2))),
            int(round(y + (height / 2))))


def warp_image(image: np.ndarray, matrix: np.ndarray, width: int, height: int) -> np.ndarray:
    """Warp the image based on the transformation matrix."""
    return cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_LINEAR)


def merge_image(base:np.ndarray, overlay:np.ndarray, x1:int, y1:int, x2:int, y2:int) -> np.ndarray:
    """Merge two images."""
    base[y1:y2, x1:x2] = overlay
    return base


def stitch_images(left: np.ndarray, center: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Stitch the images together."""
    center = cv2.resize(center, (MAX_WIDTH, MAX_HEIGHT))
    left = cv2.resize(left, (MAX_WIDTH, MAX_HEIGHT))
    right = cv2.resize(right, (MAX_WIDTH, MAX_HEIGHT))

    # Warp images
    left_res = warp_image(left, MATRIX_LEFT, LEFT_WIDTH, LEFT_HEIGHT)
    right_res = warp_image(right, MATRIX_RIGHT, RIGHT_WIDTH, RIGHT_HEIGHT)

    # Calculate result image size
    result_width = int(math.ceil(LEFT_WIDTH / 2 / LEFT_X))
    result_height = int(math.ceil(MAX_HEIGHT / 2 / CENTER_Y))

    # Calculate the position of the images
    lxc, lyc = relative_to_absolute(LEFT_X, LEFT_Y, result_width, result_height)
    lx1, ly1, lx2, ly2 = get_ltbr(lxc, lyc, LEFT_WIDTH, LEFT_HEIGHT)

    rxc, ryc = relative_to_absolute(RIGHT_X, RIGHT_Y, result_width, result_height)
    rx1, ry1, rx2, ry2 = get_ltbr(rxc, ryc, RIGHT_WIDTH, RIGHT_HEIGHT)

    cxc, cyc = relative_to_absolute(CENTER_X, CENTER_Y, result_width, result_height)
    cx1, cy1, cx2, cy2 = get_ltbr(cxc, cyc, MAX_WIDTH, MAX_HEIGHT)

    # Create result image
    result = np.zeros((result_height, result_width, 3), dtype=np.uint8)
    result = merge_image(result, left_res, lx1, ly1, lx2, ly2)
    result = merge_image(result, right_res, rx1, ry1, rx2, ry2)
    return merge_image(result, center, cx1, cy1, cx2, cy2)


if __name__ == "__main__":
    # Load images
    center_img = cv2.imread("images/crossing/center.jpg")
    left_img = cv2.imread("images/crossing/left.jpg")
    right_img = cv2.imread("images/crossing/right.jpg")

    # Write result
    result_img = stitch_images(left_img, center_img, right_img)
    cv2.imwrite("images/result.jpg", result_img)
