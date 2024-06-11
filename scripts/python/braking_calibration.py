import logging
import time

from typing import Any

from src.driving.can import CANController, get_can_bus
from src.driving.gamepad import EventType, Gamepad, GamepadAxis, GamepadButton
from src.driving.modes import ManualDriving


config = {"braking_force_min": 0, "braking_force_max": 100, "braking_force_step": 5}


class BrakeCalibrationProcedure:
    """Procedure to calibrate the braking force.

    This procedure will calibrate the braking force. It will do this by
    applying the brakes and checking if the wheels lock up. If they do we
    will lower the amount of braking force. If they don't we will increase
    the amount of braking force. This will be done using a binary search
    algorithm.

    Attributes
    ----------
        can_controller (CANController): The controller for the CAN bus.
        gamepad (Gamepad): The gamepad to use.
        braking_steps (list[int]): The steps to use for the braking force.
        low_idx (int): The lowest index.
        high_idx (int): The highest index.
        middle_idx (int): The middle index.
        locked (bool): Whether the wheels are locked.
        braked (bool): Whether the brakes have been applied.

    """

    low_idx: int
    high_idx: int
    middle_idx: int

    locked: bool = False
    braked: bool = False
    started: bool = False

    def __init__(self, can_controller: CANController, gamepad: Gamepad) -> None:
        """Procedure to calibrate the braking force."""
        self.can_controller = can_controller
        self.gamepad = gamepad

        # register the buttons
        self.gamepad.add_listener(GamepadButton.RB, EventType.BUTTON_DOWN, self.start_procedure)
        self.braking_steps = list(
            range(config["braking_force_min"], config["braking_force_max"] + 1, config["braking_force_step"])
        )

        self.low_idx = 0
        self.high_idx = len(self.braking_steps) - 1
        self.middle_idx = self.low_idx + (self.high_idx - self.low_idx) // 2

    def start_procedure(self, *_args: Any, **_kwargs: Any) -> None:
        """Start the brake calibration procedure.

        This will register all needed callbacks. Next to this it often won't do
        anything else. This function should be called when the B button is pressed.
        """
        if self.started:
            logging.warning("brake calibration already started")
            return

        logging.info("start brake calibration")
        self.gamepad.add_listener(GamepadAxis.DPAD_X, EventType.AXIS_CHANGED, self.__arrow_pressed)
        self.gamepad.add_listener(GamepadAxis.DPAD_Y, EventType.AXIS_CHANGED, self.__arrow_pressed)
        self.gamepad.add_listener(GamepadButton.LB, EventType.BUTTON_DOWN, self.__start_braking)
        self.gamepad.add_listener(GamepadButton.LB, EventType.BUTTON_UP, self.__stop_braking)

        self.started = True

    def __arrow_pressed(self, button: GamepadAxis, _event: EventType, val: float) -> None:
        """Event listener for the arrow buttons.

        :param button: The button that was pressed.
        :param val: The value of the button.
        """
        if val == 0:
            return

        if button == GamepadAxis.DPAD_Y:
            if val == 1:
                logging.info("select lockup")
                self.locked = True
            elif val == -1:
                logging.info("select non lockup")
                self.locked = False
        if button == GamepadAxis.DPAD_X:
            logging.info("confirm")
            if val == 1:
                self.__confirm_lockup()

    def __confirm_lockup(self) -> None:
        """Event listener to confirm the lockup.

        If this function is called we will adjust the range of braking force
        according to binary sort. If we locked up we will lower the amount.
        If we didn't lock up we will increase the amount.
        """
        # check if we have braked.
        if not self.braked:
            logging.info("brake first")
            return

        self.braked = False

        with open("braking_force.txt", "w") as f:
            if self.low_idx >= self.high_idx:
                f.write(f"braking force calibrated: {self.braking_steps[self.middle_idx]}")
                logging.info("braking force calibrated: %d", self.braking_steps[self.middle_idx])

            f.write(f"low: {self.low_idx} high: {self.high_idx} middle: {self.middle_idx}")
            logging.info("low: %d high: %d middle: %d", self.low_idx, self.high_idx, self.middle_idx)
            if self.locked:
                self.high_idx = self.middle_idx - 1
            else:
                self.low_idx = self.middle_idx + 1

        # adjust for the next iteration
        self.locked = False
        self.middle_idx = self.low_idx + (self.high_idx - self.low_idx) // 2

    def __start_braking(self, *_args: Any, **_kwargs: Any) -> None:
        """Start braking at the set pressure."""
        self.can_controller.set_brake(self.braking_steps[self.middle_idx])

    def __stop_braking(self, *_args: Any, **_kwargs: Any) -> None:
        """Stop braking."""
        self.can_controller.set_brake(0)
        self.braked = True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    can_bus = get_can_bus()
    can_controller = CANController(can_bus)
    can_controller.start()

    gamepad = Gamepad()
    gamepad.start()

    controller_driving = ManualDriving(gamepad, can_controller)
    controller_driving.start()

    brake_calibration = BrakeCalibrationProcedure(can_controller, gamepad)

    while True:
        time.sleep(1)
