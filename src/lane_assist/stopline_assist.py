from config import config
from driving.speed_controller import ISpeedController, SpeedControllerState
from utils.calibration_data import CalibrationData


class StopLineAssist:
    """Class to assist stopping at stop lines.

    This class will assist in stopping at stop lines.

    TODOS:
    ------
      - TODO: creep towards the stop line if stopped short of it.
      - TODO: take into account the speed of the kart.

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

    def handle_stop_lines(self, stop_lines: list[int]) -> None:
        """Handle the stop lines.

        This function will handle the stop lines in the image.
        If a stop line is found, it will stop the kart.

        :param stop_lines: the stop lines found in the image.
        """
        if len(stop_lines) == 0:
            self.stop_lines_found = max(0, self.stop_lines_found - 1)
            return

        if self.stop_lines_found < 3:
            self.stop_lines_found += 1
            return

        if self.speed_controller.state != SpeedControllerState.WAITING_TO_STOP:
            return

        height = self.__calibration.output_shape[1]
        distance = self.__calibration.get_distance(height - stop_lines[0])

        braking_distance = self.speed_controller.get_braking_distance()
        total_distance = distance - braking_distance

        if total_distance > config.traffic_light.min_distance:
            return

        self.speed_controller.state = SpeedControllerState.STOPPED
        self.stop_lines_found = 0
