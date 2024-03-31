from driving.speed_controller import SpeedControllerState
from driving.speed_controller.speed_controller_interface import ISpeedController


class StoplineAssist:
    """Class to assist stopping at stoplines.

    This class will assist in stopping at stoplines.

    TODOS:
    ------
      - TODO: creep towards the stopline if stopped short of it.
      - TODO: take into account the speed of the kart.

    :param speed_controller: the speed controller to use for stopping at stoplines.
    :param stop_lines_found: the amount of frames a stopline has been detected
    """

    speed_controller: ISpeedController
    stop_lines_found: int

    def __init__(self, speed_controller: ISpeedController) -> None:
        """Initialize the stopline assist.

        :param speed_controller: the speed controller to use for stopping at stoplines.
        """
        self.speed_controller = speed_controller
        self.stop_lines_found = 0

    def handle_stoplines(self, stoplines: list[int]) -> None:
        """Handle the stoplines.

        This function will handle the stoplines in the image.
        If a stopline is found, it will stop the kart.

        :param stoplines: the stoplines found in the image.
        """
        if len(stoplines) == 0:
            self.stop_lines_found = max(0, self.stop_lines_found - 1)
            return

        if self.stop_lines_found < 3:
            self.stop_lines_found += 1
            return

        # TODO: take the distance to the stopline and speed into account
        if self.speed_controller.state == SpeedControllerState.WAITING_TO_STOP and stoplines[0] < 500:
            self.speed_controller.state = SpeedControllerState.STOPPED
            self.stop_lines_found = 0
