from driving.can_controller.can_controller_interface import ICANController


class MockCanController(ICANController):
    """A mock CAN controller for testing purposes."""

    def add_listener(self, message_id: int, listener: callable) -> None:
        """Add a listener for a message."""
        pass

    def set_brake(self, brake: int) -> None:
        """Set the brake of the go-kart."""
        pass

    def set_steering(self, angle: float) -> None:
        """Set the steering of the go-kart."""
        pass

    def set_throttle(self, throttle: int, gear: int) -> None:
        """Set the throttle of the go-kart."""
        pass

    def start(self) -> None:
        """Start the CAN controller."""
        pass

