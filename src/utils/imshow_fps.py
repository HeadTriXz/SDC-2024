import time

import cv2
import numpy as np

last_frame = {}


def imshow_fps(name: str, img: np.ndarray) -> None:
    """Show an image with the fps in the title."""
    if name not in last_frame:
        last_frame[name] = time.time()

        fps = 1 / (time.time() - last_frame[name])
        # add the fps to the title
        cv2.setWindowTitle(name, f"{name} - {fps:.2f} fps")

    cv2.imshow(name, img)
    cv2.waitKey(1)
