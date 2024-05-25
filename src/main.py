import cv2
import logging

from src.calibration.data import CalibrationData
from src.lane_assist.preprocessing.image_filters import morphex_filter


if __name__ == "__main__":
    image_names = [f"./data/images/topdown/{i}.jpg" for i in range(1, 22)]
    images = [cv2.imread(file, cv2.IMREAD_GRAYSCALE) for file in image_names]

    calibration = CalibrationData.load("./data/calibration/latest.npz")

    for i, image in enumerate(images):
        logging.debug("showing image %s", image_names[i])
        cv2.imshow("original", image)

        # filter the image
        filter_mask = cv2.bitwise_not(morphex_filter(image, calibration))
        filtered_image = cv2.bitwise_and(image, filter_mask)
        cv2.imshow("mask", filter_mask)
        cv2.imshow("filtered", filtered_image)

        while cv2.waitKey(1) != ord("q"):
            pass
