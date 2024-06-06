import airsim
import can

from typing import Callable

from src.constants import CANFeedbackIdentifier, Gear
from src.driving.can import ICANController


class SimCanController(ICANController):
    """Simulate the can controller.

    Attributes
    ----------
        get_client: The client to get the data.
        update_client: The client to update the data.

    """

    get_client: airsim.CarClient
    update_client: airsim.CarClient

    __brake: float = 0.0
    __gear: Gear = Gear.NEUTRAL
    __throttle: float = 0.0
    __steering: float = 0.0

    __updating: bool = False
    __listeners: dict[CANFeedbackIdentifier, list[Callable[[can.Message], None]]]

    def __init__(self, autonomous: bool = True) -> None:
        """Initialize the can controller.

        :param autonomous: Whether the vehicle is driving autonomously.
        """
        self.update_client = airsim.CarClient()
        self.update_client.confirmConnection()

        self.update_client.enableApiControl(autonomous)
        if autonomous:
            self.update_client.reset()

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
        self.__brake = brake / 100
        self.update()

    def set_throttle(self, throttle: int, gear: Gear) -> None:
        """Set the speed."""
        self.__throttle = throttle / 100
        self.__gear = gear
        self.update()

    def set_steering(self, steering: float) -> None:
        """Set the steering."""
        self.__steering = steering
        self.update()

    def start(self) -> None:
        """Start the controller."""
        pass

    def update(self) -> None:
        """Update the controller."""
        car_controls = airsim.CarControls()
        car_controls.brake = self.__brake
        car_controls.steering = self.__steering
        car_controls.throttle = self.__throttle
        car_controls.is_manual_gear = self.__gear == Gear.REVERSE
        if car_controls.is_manual_gear:
            car_controls.manual_gear = -1

        self.update_client.setCarControls(car_controls)

        state = self.update_client.getCarState()
        self.__update(state.speed)

    def __update(self, speed: float) -> None:
        if self.__updating:
            return

        self.__updating = True

        speed = max(0, int(speed * 36))
        msg_bytes = speed.to_bytes(2, byteorder="big")

        can_msg = can.Message(arbitration_id=CANFeedbackIdentifier.SPEED_SENSOR, data=msg_bytes)
        if CANFeedbackIdentifier.SPEED_SENSOR in self.__listeners:
            for listener in self.__listeners[CANFeedbackIdentifier.SPEED_SENSOR]:
                listener(can_msg)

        self.__updating = False
