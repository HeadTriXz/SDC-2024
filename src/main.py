import logging
import os

import cv2

from src.calibration.data import CalibrationData
from src.lane_assist.line_detection.line_detector import get_lines

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from src.simulation.main import start_simulator

        start_simulator()
    else:
        img = cv2.imread("download.jpg", cv2.IMREAD_GRAYSCALE)
        calibration = CalibrationData.load("./data/calibration/latest.npz")
        # threshold the img
        cv2.threshold(img, 127, 255, cv2.THRESH_BINARY, img)

        lines = get_lines(img, calibration)

    #
    # if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
    #     from src.simulation.main import start_simulator
    #
    #     start_simulator()
    # else:
    #     # Kart().start()
    #
    #     bus = get_can_bus()
    #     can = CANController(bus)
    #     can.start()
    #
    #     kart = AutonomousDriving(can)
    #     kart.toggle()
    #     kart.start()
