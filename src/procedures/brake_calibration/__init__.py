import logging

import config
from kart_control.can_controller import CANController
from kart_control.new_controller import (
    Controller,
    ControllerAxis,
    ControllerButton,
    EventType,
)


class BrakeCalibrationProcedure:
    """Procedure to calibrate the braking force.

    this procedure will calibrate the braking force. it will do this by
    applying the brakes and checking if the wheels lock up. if they do we
    will lower the amount of braking force. if they don't we will increase
    the amount of braking force. this will be done using a binary search
    algorithm.
    """

    low_idx: int
    high_idx: int
    middle_idx: int

    locked: bool = False
    braked: bool = False

    def __init__(self, driving: CANController, controller: Controller) -> None:
        """Procedure to calibrate the braking force."""
        self.driving = driving
        self.controller = controller

        # register the buttons
        self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.RB, self.start_procedure)
        self.braking_steps = list(
            range(
                config.braking_force_min,  # start
                config.braking_force_max + 1,  # stop. +1 because range is exclusive
                config.braking_force_step,  # step
            )
        )

        self.high_idx = len(self.braking_steps) - 1
        self.low_idx = 0
        self.middle_idx = self.low_idx + (self.high_idx - self.low_idx) // 2

        self.logger = logging.getLogger(__name__)

    def start_procedure(self, _event: EventType, _button: ControllerButton) -> None:
        """Start the brake calibration procedure.

        this will register all needed callbacks. next to this it often won't do
        anything else. this function should be called when the B button is pressed.
        """
        self.logger.info("start brake calibration")
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.DPAD_X, self.__arrow_pressed)
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.DPAD_Y, self.__arrow_pressed)
        self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.LB, self.__start_braking)
        self.controller.add_listener(EventType.BUTTON_UP, ControllerButton.LB, self.__stop_braking)

    def __arrow_pressed(self, _event: EventType, button: ControllerAxis, val: float) -> None:
        if val == 0:
            return

        if button == ControllerAxis.DPAD_Y:
            if val == 1:
                self.logger.info("select lockup")
                self.locked = True
            elif val == -1:
                self.logger.info("select non lockup")
                self.locked = False
        if button == ControllerAxis.DPAD_X:
            self.logger.info("confirm")
            if val == 1:
                self.__confirm_lockup()

    def __confirm_lockup(self) -> None:
        """Event listener to confirm the lockup.

        if this function is called we will adjust the range of braking force
         according to binary sort. if we locked up we will lower the amount.
         if we didn't lock up we will increase the amount.
        """
        # check if we have braked.
        if not self.braked:
            self.logger.info("brake first")
            return

        self.braked = False

        with open("braking_force.txt", "w") as f:
            if self.low_idx >= self.high_idx:
                f.write(f"braking force calibrated: {self.braking_steps[self.middle_idx]}")
                self.logger.info("braking force calibrated: %d", self.braking_steps[self.middle_idx])

            f.write(f"low: {self.low_idx} high: {self.high_idx} middle: {self.middle_idx}")
            self.logger.info("low: %d high: %d middle: %d", self.low_idx, self.high_idx, self.middle_idx)
            if self.locked:
                self.high_idx = self.middle_idx - 1
            else:
                self.low_idx = self.middle_idx + 1

        # check if we are done

        # adjust for the next iteration
        self.locked = False
        self.middle_idx = self.low_idx + (self.high_idx - self.low_idx) // 2

    def __start_braking(self, _event: EventType, _button: ControllerButton) -> None:
        """Start braking at the set pressure."""
        self.driving.set_brake(self.braking_steps[self.middle_idx])

    def __stop_braking(self, _event: EventType, _button: ControllerButton) -> None:
        """Stop braking."""
        self.driving.set_brake(0)
        self.braked = True
