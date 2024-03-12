import cv2
import numpy as np
import time

def cut_image(image, x, y, width, height):
    return image[y:y + height, x:x + width]

def topdown(image, image_width, image_height):
    if image is None:
        raise ValueError("Error: Unable to load image")

    DIVISOR_WIDTH = 3970
    DIVISOR_HEIGHT = 1971

    WIDTH_RATIO = 1 / DIVISOR_WIDTH
    HEIGHT_RATIO = 1 / DIVISOR_HEIGHT

    pts = np.array([[55 * WIDTH_RATIO * image_width, 900 * HEIGHT_RATIO * image_height],
                    [1841 * WIDTH_RATIO * image_width, 253 * HEIGHT_RATIO * image_height],
                    [2067 * WIDTH_RATIO * image_width, 253 * HEIGHT_RATIO * image_height],
                    [3861 * WIDTH_RATIO * image_width, 900 * HEIGHT_RATIO * image_height]], dtype=np.float32)

    ipm_pts = np.array([[600 * WIDTH_RATIO * image_width, 1100 * HEIGHT_RATIO * image_height],
                        [620 * WIDTH_RATIO * image_width, 450 * HEIGHT_RATIO * image_height],
                        [780 * WIDTH_RATIO * image_width, 450 * HEIGHT_RATIO * image_height],
                        [800 * WIDTH_RATIO * image_width, 1100 * HEIGHT_RATIO * image_height]], dtype=np.float32)

    ipm_matrix = cv2.getPerspectiveTransform(pts, ipm_pts)
    ipm = cv2.warpPerspective(image, ipm_matrix, (image_width, image_height), flags=cv2.INTER_NEAREST)
    ipm = cut_image(ipm, int(300 * WIDTH_RATIO * image_width), int(200 * HEIGHT_RATIO * image_height), int(800 * WIDTH_RATIO * image_width), int(900 * HEIGHT_RATIO * image_height))

    return ipm
