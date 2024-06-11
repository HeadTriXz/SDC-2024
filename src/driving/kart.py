import time

from typing import Any

from src.driving.can import CANController, get_can_bus
from src.driving.gamepad import EventType, Gamepad, GamepadButton
from src.driving.modes import AutonomousDriving, DrivingMode, ManualDriving


class Kart:
    """The main class for the system. This class will start the kart and all its components.

    Attributes
    ----------
        autonomous (AutonomousDriving): The autonomous driving mode.
        manual (BasicControllerDriving): The manual driving mode.

    """

    autonomous: AutonomousDriving
    manual: DrivingMode

    __autonomous: bool = False
    __can: CANController
    __gamepad: Gamepad

    def __init__(self) -> None:
        """Initialize the kart."""
        bus = get_can_bus()
        self.__can = CANController(bus)
        self.__gamepad = Gamepad()

        self.autonomous = AutonomousDriving(self.__can)
        self.manual = ManualDriving(self.__gamepad, self.__can)

        self.__gamepad.add_listener(GamepadButton.START, EventType.LONG_PRESS, self.__toggle_delayed)
        self.__gamepad.add_listener(GamepadButton.SELECT, EventType.LONG_PRESS, self.__toggle_delayed)
        self.__gamepad.add_listener(GamepadButton.LB, EventType.BUTTON_DOWN, self.__toggle_can_recording)

        self.autonomous.telemetry.add_callback_function("toggle_driving_mode", self.__toggle)

    def start(self) -> "Kart":
        """Start the kart and all its components."""
        self.__can.start()
        self.__gamepad.start()

        self.manual.start()
        self.autonomous.start()

        return self

    def __toggle_can_recording(self, *_args: Any, **_kwargs: Any) -> None:
        """Toggle the recording of CAN messages."""
        self.__can.toggle_recording()
        self.__gamepad.vibrate()

    def __toggle_delayed(self, *_args: Any, **_kwargs: Any) -> None:
        """Toggle between autonomous and manual driving with a delay."""
        if not self.__autonomous:
            for _ in range(3):
                self.__gamepad.vibrate(300)
                time.sleep(2)

        self.__toggle()

    def __toggle(self, *_args: Any, **_kwargs: Any) -> None:
        """Toggle between autonomous and manual driving."""
        self.__autonomous = not self.__autonomous

        self.autonomous.toggle()
        self.manual.toggle()
        self.__gamepad.vibrate()
