import cv2
import numpy as np


def cut_image(image, x, y, width, height):
    return image[y:y + height, x:x + width]


def resize_image(image, target_width, target_height):
    resized_image = cv2.resize(image, (target_width, target_height))
    return resized_image


def topdown(image):
    """ a function to transform the image to a top-down view of the road
    Args:
        image: the image to be transformed
    """
    img = image
    if img is None:
        print("Error: Unable to load image.")
        exit()

    pts = np.array([[55, 900], [1841, 253], [2067, 253], [3861, 900]], dtype=np.float32)
    ipm_pts = np.array([[800, 450], [800, 1100], [600, 1100], [600, 450]], dtype=np.float32)
    ipm_matrix = cv2.getPerspectiveTransform(pts, ipm_pts)
    ipm = cv2.warpPerspective(img, ipm_matrix, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)
    ipm = cut_image(ipm, 300, 450, 800, 900)
    ipm = cv2.rotate(ipm, cv2.ROTATE_90_CLOCKWISE)
    ipm = cv2.rotate(ipm, cv2.ROTATE_90_CLOCKWISE)
    return ipm
