import airsim
from driving.can.can_controller_interface import ICANController


class SimCanController(ICANController):
    """Simulate the can controller."""

    def __init__(self, client: airsim.CarClient) -> None:
        """Initialize the can controller."""
        self.client = client
        self.brake = 0
        self.throttle = 0
        self.steering = 0

    def add_listener(self, message_id: int, listener: callable) -> None:
        """Add a listener."""
        pass

    def set_brake(self, brake: int) -> None:
        """Set the brake."""
        self.brake = brake / 100
        self.update()

    def set_throttle(self, throttle: int, _gear: int) -> None:
        """Set the speed."""
        self.throttle = throttle / 100
        self.update()

    def set_steering(self, steering: float) -> None:
        """Set the steering."""
        self.steering = steering
        self.update()

    def start(self) -> None:
        """Start the controller."""
        pass

    def update(self) -> None:
        """Update the controller."""
        car_controls = airsim.CarControls()
        car_controls.steering = self.steering
        car_controls.throttle = self.throttle
        car_controls.brake = self.brake
        self.client.setCarControls(car_controls)
