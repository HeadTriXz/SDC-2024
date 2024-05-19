import airsim
import can

from src.constants import CANFeedbackIdentifier
from src.driving.can import ICANController


class SimCanController(ICANController):
    """Simulate the can controller."""

    updating = False

    brake = 0
    throttle = 0
    steering = 0

    def __init__(self) -> None:
        """Initialize the can controller."""
        self.update_client = airsim.CarClient()
        self.update_client.confirmConnection()
        self.update_client.enableApiControl(False)
        # self.update_client.reset()

        self.get_client = airsim.CarClient()
        self.get_client.confirmConnection()

        self.__listeners = {}

    def add_listener(self, message_id: CANFeedbackIdentifier, listener: callable) -> None:
        """Add a listener."""
        if message_id not in self.__listeners:
            self.__listeners[message_id] = []
        self.__listeners[message_id].append(listener)

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
        self.update_client.setCarControls(car_controls)
        state = self.update_client.getCarState()
        self.__update(state.speed)

    def __update(self, speed: float) -> None:
        if self.updating:
            return

        self.updating = True

        speed = max(0, int(speed * 36))
        msg_bytes = speed.to_bytes(2, byteorder="big")

        can_msg = can.Message(arbitration_id=CANFeedbackIdentifier.SPEED_SENSOR, data=msg_bytes)
        if CANFeedbackIdentifier.SPEED_SENSOR in self.__listeners:
            for listener in self.__listeners[CANFeedbackIdentifier.SPEED_SENSOR]:
                listener(can_msg)

        self.updating = False
