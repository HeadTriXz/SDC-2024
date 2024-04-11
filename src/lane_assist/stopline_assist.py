from driving.speed_controller import ISpeedController, SpeedControllerState


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

    def __init__(self, speed_controller: ISpeedController) -> None:
        """Initialize the stop line assist.

        :param speed_controller: the speed controller to use for stopping at stop-lines.
        """
        self.speed_controller = speed_controller

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

        # TODO: take the distance to the stopline and speed into account
        if self.speed_controller.state == SpeedControllerState.WAITING_TO_STOP and 200 < stop_lines[0] < 700:
            self.speed_controller.state = SpeedControllerState.STOPPED
            self.stop_lines_found = 0
