import logging

from typing import Any

from src.constants import Gear
from src.driving.can import ICANController
from src.driving.gamepad import (
    EventType,
    Gamepad,
    GamepadAxis,
    GamepadButton,
)
from src.driving.modes import DrivingMode


class ManualDriving(DrivingMode):
    """A basic driving controller using a gamepad.

    Attributes
    ----------
        can_controller (CANController): The CAN controller to use.
        gamepad (Controller): The gamepad to use.
        gear (Gear): The current gear of the vehicle.

    """

    can_controller: ICANController
    gamepad: Gamepad
    gear = Gear.NEUTRAL

    __running: bool = True
    __primed: bool = False

    def __init__(self, gamepad: Gamepad, can_controller: ICANController) -> None:
        """Initialize the driving controller.

        :param gamepad: The gamepad to use.
        :param can_controller: The CAN controller to use.
        """
        super().__init__()
        self.can_controller = can_controller
        self.gamepad = gamepad

    def start(self) -> None:
        """Start driving using a controller."""
        logging.info("Starting driving controller")

        self.can_controller.set_throttle(0, Gear.NEUTRAL)
        self.can_controller.set_brake(100)

        self.gamepad.add_listener(GamepadButton.A, EventType.LONG_PRESS, self.__ready)
        self.gamepad.vibrate()

    def toggle(self) -> None:
        """Toggle the controller."""
        self.__running = not self.__running

    def __ready(self, *_args: Any, **_kwargs: Any) -> None:
        """The controller is ready to drive."""
        if self.__primed:
            return

        self.__primed = True

        logging.info("The controller is ready")
        self.gamepad.vibrate()

        # gears
        self.gamepad.add_listener(GamepadButton.X, EventType.BUTTON_DOWN, self.__set_reverse)
        self.gamepad.add_listener(GamepadButton.Y, EventType.BUTTON_DOWN, self.__set_neutral)
        self.gamepad.add_listener(GamepadButton.B, EventType.BUTTON_DOWN, self.__set_forward)

        # throttle and brake
        self.gamepad.add_listener(GamepadAxis.RT, EventType.AXIS_CHANGED, self.__set_throttle)
        self.gamepad.add_listener(GamepadAxis.LT, EventType.AXIS_CHANGED, self.__set_brake)

        # steering
        self.gamepad.add_listener(GamepadAxis.LS_X, EventType.AXIS_CHANGED, self.__set_steering)

    def __set_brake(self, _button: GamepadButton, _event: EventType, value: float) -> None:
        """Update the brake value.

        :param value: The value of the brake.
        """
        if not self.__running:
            return

        self.can_controller.set_brake(int(value * 100))

    def __set_forward(self, *_args: Any, **_kwargs: Any) -> None:
        """Set the gear to forward."""
        if not self.__running:
            return

        self.gear = Gear.DRIVE

    def __set_neutral(self, *_args: Any, **_kwargs: Any) -> None:
        """Set the gear to neutral."""
        if not self.__running:
            return

        self.gear = Gear.NEUTRAL

    def __set_reverse(self, *_args: Any, **_kwargs: Any) -> None:
        """Set the gear to reverse."""
        if not self.__running:
            return

        self.gear = Gear.REVERSE

    def __set_steering(self, _button: GamepadButton, _event: EventType, value: float) -> None:
        """Update the steering angle.

        :param value: The value of the steering angle.
        """
        if not self.__running:
            return

        if abs(value) <= 0.1:
            value = 0.0

        self.can_controller.set_steering(value)

    def __set_throttle(self, _button: GamepadButton, _event: EventType, value: float) -> None:
        """Update the throttle value.

        :param value: The value of the throttle.
        """
        if not self.__running:
            return

        self.can_controller.set_throttle(int(value * 100), self.gear)