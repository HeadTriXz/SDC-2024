import threading
from enum import IntEnum

import can

from common import config
from common.constants import CANFeedbackIdentifier, Gear
from new_controller.CANController import CANController


class SpeedControllerState(IntEnum): 
    """The states the speed controller can be in."""

    STOPPED = 0
    WAITING_TO_STOP = 1
    DRIVING = 2

class SpeedController:
    """A controller for the speed of the go-kart.

    Attributes
    ----------
        current_speed (float): The current speed of the go-kart.
        gear (Gear): The gear of the go-kart.
        max_speed (int): The maximum speed of the go-kart.
        target_speed (int): The target speed of the go-kart.
        stopped (bool): Whether the go-kart is stopped.
    
    """

    current_speed: float
    gear: Gear
    satte: SpeedControllerState

    __can: CANController
    __max_speed: int
    __target_speed: int
    __thread: threading.Thread

    def __init__(self, can_bus: CANController) -> None:
        """Initialize the speed controller.

        :param can_bus: The CAN controller to use.
        """
        self.current_speed = 0
        self.gear = Gear.NEUTRAL
        self.state = SpeedControllerState.STOPPED

        self.__can = can_bus
        self.__max_speed = 0
        self.__target_speed = 0
        self.__thread = threading.Thread(target=self.__listen, daemon=True)

    @property
    def max_speed(self) -> int:
        """The maximum speed of the go-kart."""
        return self.__max_speed

    @max_speed.setter
    def max_speed(self, speed: int) -> None:
        """Set the maximum speed of the go-kart."""
        if speed < 0:
            raise ValueError("The maximum speed cannot be negative.")

        if speed < self.__target_speed:
            self.__target_speed = speed

        self.__max_speed = speed

    @property
    def target_speed(self) -> int:
        """The target speed of the go-kart."""
        return self.__target_speed

    @target_speed.setter
    def target_speed(self, speed: int) -> None:
        """Set the target speed of the go-kart."""
        if speed < 0:
            raise ValueError("The target speed cannot be negative.")

        if speed > self.__max_speed:
            speed = self.__max_speed
            print("The target speed cannot be greater than the maximum speed.")  # noqa: T201 TODO: change to use logger

        self.__target_speed = speed

    def start(self) -> None:
        """Start the speed controller."""
        self.__can.add_listener(CANFeedbackIdentifier.SPEED_SENSOR, self.__update_speed)
        self.__thread.start()

    def __get_target_percentage(self) -> int:
        """Get the target percentage of the throttle to apply."""
        return int(config.speed / self.__target_speed * 100)

    def __listen(self) -> None:
        """Listen for changes in the speed of the go-kart."""
        while True:
            if self.state == SpeedControllerState.STOPPED:
                self.__can.set_throttle(0, Gear.NEUTRAL)
                self.__can.set_brake(100) # TODO: change brake value.
                continue

            self.__can.set_throttle(self.__get_target_percentage(), self.gear)

            if self.current_speed < self.__target_speed:
                self.__can.set_brake(0)
            else:
                self.__can.set_brake(30) # TODO: Dynamically calculate required braking force.

    def __update_speed(self, message: can.Message) -> None:
        """Update the speed of the go-kart."""
        value = int.from_bytes(message.data[:2], byteorder="big")
        self.current_speed = value / 10
