import math

import cv2
import numpy as np

# Constants
MIN_HEIGHT = 0
MIN_WIDTH = 0
MAX_HEIGHT = 720
MAX_WIDTH = 1280

PTS_ORIGINAL = np.float32([[MIN_WIDTH, MIN_HEIGHT], [MIN_WIDTH, MAX_HEIGHT],
                           [MAX_WIDTH, MAX_HEIGHT], [MAX_WIDTH, MIN_HEIGHT]])

RATIOS_LEFT = [[0, 1.0055555], [0.36197916, 2.6185186],
               [1.6765625, 0.7537037], [1.5010417, 0]]
RATIOS_RIGHT = [[0.18125, 0], [0, 0.74907407],
                [1.28125, 2.55833333], [1.66770833, 0.99722222]]

LEFT_X, LEFT_Y = 0.27027708, 0.52165085
RIGHT_X, RIGHT_Y = 0.73115027, 0.51505412
CENTER_X, CENTER_Y = 0.5049538, 0.1826793

# Functions
def get_points(ratio) -> np.ndarray:
    """ Get the points of the warped image based on the ratio. """
    return np.float32(ratio * np.array([MAX_WIDTH, MAX_HEIGHT]))

def get_matrix(ratio) -> np.ndarray:
    """ Get the transformation matrix based on the ratio. """
    return cv2.getPerspectiveTransform(PTS_ORIGINAL, get_points(ratio))

def get_width(ratio) -> int:
    """ Get the width of the warped image based on the ratio. """
    return int(np.max(get_points(ratio)[:, 0]))

def get_height(ratio) -> int:
    """ Get the height of the warped image based on the ratio. """
    return int(np.max(get_points(ratio)[:, 1]))

def relative_to_absolute(x, y, width, height) -> (float, float):
    """ Convert relative coordinates to absolute coordinates. """
    return x * width, y * height

def get_ltbr(x, y, width, height) -> (int, int, int, int):
    """ Get the left, top, right, and bottom coordinates of the image based on xywh. """
    return (int(round(x - (width / 2))),
            int(round(y - (height / 2))),
            int(round(x + (width / 2))),
            int(round(y + (height / 2))))

def warp_image(image, matrix, width, height) -> np.ndarray:
    """ Warp the image based on the transformation matrix. """
    return cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=[0, 0, 0, 0])

def merge_image(base, overlay, x1, y1, x2, y2):
    """ Merge two images. """
    image = base.copy()
    if overlay.shape[2] == 4:
        alpha = overlay[:, :, 3]
        mask = alpha == 255

        image[y1:y2, x1:x2][mask] = overlay[mask][:, :3]
    else:
        image[y1:y2, x1:x2] = overlay

    return image

# Main
matrix_left = get_matrix(RATIOS_LEFT)
left_width = get_width(RATIOS_LEFT)
left_height = get_height(RATIOS_LEFT)

matrix_right = get_matrix(RATIOS_RIGHT)
right_width = get_width(RATIOS_RIGHT)
right_height = get_height(RATIOS_RIGHT)

def stitch_images(left: np.ndarray, center: np.ndarray, right: np.ndarray) -> np.ndarray:
    """ Stitch the images together. """
    center = cv2.resize(center, (MAX_WIDTH, MAX_HEIGHT))
    left = cv2.resize(left, (MAX_WIDTH, MAX_HEIGHT))
    right = cv2.resize(right, (MAX_WIDTH, MAX_HEIGHT))

    # Warp images
    left = cv2.cvtColor(left, cv2.COLOR_RGB2RGBA)
    right = cv2.cvtColor(right, cv2.COLOR_RGB2RGBA)
    left_res = warp_image(left, matrix_left, left_width, left_height)
    right_res = warp_image(right, matrix_right, right_width, right_height)

    # Calculate result image size
    result_width = int(math.ceil(left_width / 2 / LEFT_X))
    result_height = int(math.ceil(MAX_HEIGHT / 2 / CENTER_Y))

    # Calculate the position of the images
    lxc, lyc = relative_to_absolute(LEFT_X, LEFT_Y, result_width, result_height)
    lx1, ly1, lx2, ly2 = get_ltbr(lxc, lyc, left_width, left_height)

    rxc, ryc = relative_to_absolute(RIGHT_X, RIGHT_Y, result_width, result_height)
    rx1, ry1, rx2, ry2 = get_ltbr(rxc, ryc, right_width, right_height)

    cxc, cyc = relative_to_absolute(CENTER_X, CENTER_Y, result_width, result_height)
    cx1, cy1, cx2, cy2 = get_ltbr(cxc, cyc, MAX_WIDTH, MAX_HEIGHT)

    # Create result image
    result = np.zeros((result_height, result_width, 3), dtype=np.uint8)
    result = merge_image(result, center, cx1, cy1, cx2, cy2)
    result = merge_image(result, left_res, lx1, ly1, lx2, ly2)
    result = merge_image(result, right_res, rx1, ry1, rx2, ry2)

    return result

if __name__ == '__main__':
    # Load images
    center = cv2.imread('images/corner/center.jpg')
    left = cv2.imread('images/corner/left.jpg')
    right = cv2.imread('images/corner/right.jpg')

    # Write result
    result = stitch_images(left, center, right)
    cv2.imwrite('images/result.jpg', result)
