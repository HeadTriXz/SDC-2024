import time

import cv2
import numpy as np

last_frame = {}


def imshow_fps(name: str, img: np.ndarray) -> None:
    """Show an image with the fps in the title."""
    if name not in last_frame:
        last_frame[name] = time.perf_counter() - 1

    fps = 1 / (time.perf_counter() - last_frame[name])
    last_frame[name] = time.perf_counter()
    # add the fps to the title
    cv2.setWindowTitle(name, f"{name} - {fps:.2f} fps")

    cv2.imshow(name, img)
    cv2.waitKey(1)
