import numpy as np

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.speed_controller import ISpeedController, SpeedControllerState
from src.lane_assist.line_detection.line import Line
from src.lane_assist.line_detection.line_detector import get_stop_lines


class StopLineAssist:
    """Class to assist stopping at stop lines.

    This class will assist in stopping at stop lines.

    Attributes
    ----------
        speed_controller: The speed controller to use for stopping at stop lines.
        stop_lines_found: The amount of frames a stop line has been detected.

    """

    speed_controller: ISpeedController
    stop_lines_found: int = 0

    __calibration: CalibrationData

    def __init__(self, speed_controller: ISpeedController, calibration: CalibrationData) -> None:
        """Initialize the stop line assist.

        :param speed_controller: The speed controller to use for stopping at stop-lines.
        :param calibration: The calibration data.
        """
        self.speed_controller = speed_controller
        self.__calibration = calibration

    def detect_and_handle(self, img: np.ndarray, filtered_lines: list[Line]) -> None:
        """Handle the stop lines.

        This function will handle the stop lines in the image.
        If a stop line is found, it will stop the kart.

        :param filtered_lines: the stop lines found in the image.
        """
        if self.speed_controller.state != SpeedControllerState.WAITING_TO_STOP:
            return

        if len(filtered_lines) == 0:
            return

        stop_lines = get_stop_lines(img, filtered_lines, self.__calibration)
        if len(stop_lines) == 0:
            return

        braking_distance = self.speed_controller.get_braking_distance()

        for line in stop_lines:
            line_height = np.average(line.points[:, 1])
            distance = self.__calibration.get_distance(img.shape[0] - line_height)
            total_distance = distance - braking_distance

            if total_distance > config.traffic_light.min_distance:
                continue

            self.speed_controller.state = SpeedControllerState.STOPPED
            self.stop_lines_found = 0
