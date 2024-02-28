from common.decorators import run_once
from new_controller.CANController import CANController
from new_controller.controller import (
    Controller,
    ControllerButton,
    EventType,
    ControllerAxis,
)

import config


class BrakeCalibrationProcedure:
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
        self.controller.add_listener(
            EventType.BUTTON_DOWN, ControllerButton.RB, self.start_procedure
        )
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

    @run_once
    def start_procedure(self, _event: EventType, _button: ControllerButton) -> None:
        """Start the brake calibration procedure.

        this will register all needed callbacks. next to this it often won't do
        anything else. this function should be called when the B button is pressed.
        """
        print("start brake calibration")
        self.controller.add_listener(
            EventType.AXIS_CHANGED, ControllerAxis.DPAD_X, self.__arrow_pressed
        )
        self.controller.add_listener(
            EventType.AXIS_CHANGED, ControllerAxis.DPAD_Y, self.__arrow_pressed
        )
        self.controller.add_listener(
            EventType.BUTTON_DOWN, ControllerButton.LB, self.__start_braking
        )
        self.controller.add_listener(
            EventType.BUTTON_UP, ControllerButton.LB, self.__stop_braking
        )

    def __arrow_pressed(self, event, data: (ControllerAxis, float)) -> None:
        if data[1] == 0:
            return

        if data[0] == ControllerAxis.DPAD_Y:
            print(data[1])
            if data[1] == 1:
                print("select lockup")
                self.locked = True
            elif data[1] == -1:
                print("select non lockup")
                self.locked = False
        if data[0] == ControllerAxis.DPAD_X:
            print("confirm")
            if data[1] == 1:
                self.__confirm_lockup()

    def __confirm_lockup(self) -> None:
        """Event listener to confirm the lockup.

        if this function is called we will adjust the range of braking force
         according to binary sort. if we locked up we will lower the amount.
         if we didn't lock up we will increase the amount.
        """
        # check if we have braked.
        if not self.braked:
            print("brake first")
            return

        self.braked = False

        print(f"low: {self.low_idx} high: {self.high_idx} middle: {self.middle_idx}")

        print("confirm")
        if self.locked:
            self.high_idx = self.middle_idx - 1
        else:
            self.low_idx = self.middle_idx + 1

        # check if we are done
        if self.low_idx >= self.high_idx:
            print(f"braking force calibrated: {self.braking_steps[self.middle_idx]}")
            return

        print(
            f"Confirmed lockup: {self.locked} - {self.braking_steps[self.middle_idx]}"
        )

        # adjust for the next iteration
        self.locked = False
        self.middle_idx = self.low_idx + (self.high_idx - self.low_idx) // 2

    def __start_braking(self, _event, _button: ControllerButton) -> None:
        """Start braking at the set pressure."""
        self.driving.set_brake(self.braking_steps[self.middle_idx])

    def __stop_braking(self, _event, _button: ControllerButton) -> None:
        """Stop braking."""
        self.driving.set_brake(0)
        self.braked = True
