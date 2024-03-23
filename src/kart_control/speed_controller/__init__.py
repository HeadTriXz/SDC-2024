import logging
import threading
from enum import IntEnum

import can

from common import config
from common.constants import CANFeedbackIdentifier, Gear
from kart_control.can_controller import CANController


class SpeedControllerState(IntEnum):
    """States the speed controller can be in."""

    STOPPED = 0
    WAITING_TO_STOP = 1
    DRIVING = 2


class SpeedController:
    """A controller for the speed of the go-kart.

    Attributes
    ----------
        Current_speed (float): The current speed of the go-kart.
        Gear (Gear): The gear of the go-kart.
        Max_speed (int): The maximum speed of the go-kart.
        Target_speed (int): The target speed of the go-kart.
        State (SpeedControllerState): The state of the speed controller.

    """

    current_speed: float = 0
    gear: Gear = Gear.NEUTRAL

    __can: CANController
    __max_speed: int = 0
    __target_speed: int = 0
    __state: SpeedControllerState = SpeedControllerState.STOPPED

    def __init__(self, can_bus: CANController) -> None:
        """Initialize the speed controller.

        :param can_bus: The CAN controller to use.
        """
        self.__can = can_bus
        self.logger = logging.getLogger(__name__)

    def start(self) -> None:
        """Start the speed controller."""
        self.__can.add_listener(CANFeedbackIdentifier.SPEED_SENSOR, self.__update_speed)

    @property
    def state(self) -> SpeedControllerState:
        """The state of the speed controller."""
        return self.__state

    @state.setter
    def state(self, state: SpeedControllerState) -> None:
        """Set the state of the speed controller."""
        self.__state = state
        self.__adjust_speed()

    @property
    def can_controller(self) -> CANController:
        """The CAN controller."""
        return self.__can

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
        self.__adjust_speed()

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
            self.logger.warning("The target speed cannot be greater than the maximum speed.")

        self.__target_speed = speed
        self.__adjust_speed()

    def __get_target_percentage(self) -> int:
        """Get the target percentage of the throttle to apply."""
        return int(config.speed / self.__target_speed * 100)

    def __adjust_speed(self) -> None:
        """Adjust the speed of the kart.

        This function will be called when one of the following things happens:
        - the state of the kart changes
        - the target speed changes
        - the max speed changes
        - the current speed changes
        """
        # make sure the kart is stationary if it needs to be stopped
        if self.__state == SpeedControllerState.STOPPED:
            self.__can.set_throttle(0, gear=Gear.NEUTRAL)
            self.__can.set_brake(100)  # brake at the threshold
            return

        # apply throttle and brake.
        # we can always set the throttle, the brake will be set if we are driving too fast.
        self.__can.set_throttle(self.__get_target_percentage(), gear=self.gear)
        self.__can.set_brake(0 if self.current_speed <= self.__target_speed else 30)  # TODO: brake at threshold

    def __update_speed(self, message: can.Message) -> None:
        """Update the speed of the go-kart."""
        value = int.from_bytes(message.data[:2], byteorder="big")
        self.current_speed = value / 10
        self.__adjust_speed()
