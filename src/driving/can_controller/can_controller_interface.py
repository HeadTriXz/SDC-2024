from abc import ABC, abstractmethod


class ICANController(ABC):
    """An interface for a CAN controller."""

    @abstractmethod
    def add_listener(self, message_id: int, listener: callable) -> None:
        """Add a listener for a message.

        :param message_id: The identifier of the message.
        :param listener: The listener to add.
        """
        pass

    @abstractmethod
    def set_brake(self, brake: int) -> None:
        """Set the brake of the go-kart.

        :param brake: The brake to set.
        """
        pass

    @abstractmethod
    def set_steering(self, angle: float) -> None:
        """Set the steering of the go-kart.

        :param angle: The angle to set.
        """
        pass

    @abstractmethod
    def set_throttle(self, throttle: int, gear: int) -> None:
        """Set the throttle of the go-kart.

        :param throttle: The throttle to set.
        :param gear: The gear to set.
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """Start the CAN controller."""
        pass
