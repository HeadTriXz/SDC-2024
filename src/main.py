import cv2
import numpy as np

from src.calibration.data import CalibrationData
from src.config import config
from src.lane_assist.preprocessing.image_filters import morphex_filter, combined_filter

if __name__ == "__main__":

    # load the 'grayscale.mp4' video
    calibration = CalibrationData.load("./data/calibration/latest.npz")

    frame_counter = 0

    import time

    elapsed = 0

    video = cv2.VideoCapture('grayscale.mp4')
    while True:
        ret, frame = video.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        start = time.perf_counter()

        combined_filter(gray, calibration)

        end = time.perf_counter()
        elapsed += end - start
        frame_counter += 1

        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    avg = elapsed / frame_counter
    fps = frame_counter / elapsed
    print(f"FPS: {fps:.2f}, Avg: {avg:.2f}, Elapsed: {elapsed:.2f}, Frames: {frame_counter}")
