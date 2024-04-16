from abc import ABC, abstractmethod

from constants import Gear
from driving.can import ICANController
from driving.speed_controller import SpeedControllerState


class ISpeedController(ABC):
    """An interface for a speed controller."""

    @property
    @abstractmethod
    def gear(self) -> Gear:
        """The gear of the go-kart."""
        pass

    @gear.setter
    @abstractmethod
    def gear(self, gear: Gear) -> None:
        pass

    @property
    @abstractmethod
    def state(self) -> SpeedControllerState:
        """The state of the speed controller."""
        pass

    @state.setter
    @abstractmethod
    def state(self, state: SpeedControllerState) -> None:
        pass

    @property
    @abstractmethod
    def can_controller(self) -> ICANController:
        """The CAN controller."""
        pass

    @property
    @abstractmethod
    def max_speed(self) -> int:
        """The maximum speed of the go-kart."""
        pass

    @max_speed.setter
    @abstractmethod
    def max_speed(self, speed: int) -> None:
        pass

    @property
    @abstractmethod
    def target_speed(self) -> int:
        """The target speed of the go-kart."""
        pass

    @target_speed.setter
    @abstractmethod
    def target_speed(self, speed: int) -> None:
        pass

    @abstractmethod
    def get_braking_distance(self) -> float:
        """Get the braking distance of the go-kart.

        :return: The braking distance in meters.
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """Start the speed controller."""
        pass
