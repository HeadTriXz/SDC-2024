import airsim
import can
import time

from threading import Thread

from src.constants import CANFeedbackIdentifier, Gear
from src.driving.can import ICANController


class SimCanController(ICANController):
    """Simulate the can controller."""

    updating = False

    brake = 0
    gear: Gear = Gear.NEUTRAL
    throttle = 0
    steering = 0

    callbacks = []

    def __init__(self, autonomous: bool = True) -> None:
        """Initialize the can controller."""
        self.update_client = airsim.CarClient()

        self.update_client.enableApiControl(autonomous)
        if autonomous:
            self.update_client.reset()

        self.get_client = airsim.CarClient()
        self.get_client.confirmConnection()

        self.__listeners = {}

        self.thread = Thread(target=self.__run_loop)
        self.thread.start()

    def add_listener(self, message_id: CANFeedbackIdentifier, listener: callable) -> None:
        """Add a listener."""
        if message_id not in self.__listeners:
            self.__listeners[message_id] = []
        self.__listeners[message_id].append(listener)

    def set_brake(self, brake: int) -> None:
        """Set the brake."""
        self.brake = brake / 100

    def set_throttle(self, throttle: int, gear: Gear) -> None:
        """Set the speed."""
        self.throttle = throttle / 100
        self.gear = gear

    def set_steering(self, steering: float) -> None:
        """Set the steering."""
        self.steering = steering

    def start(self) -> None:
        """Start the controller."""
        pass

    def __update(self, speed: float) -> None:
        if self.updating:
            return

        self.updating = True

        speed = abs(int(speed * 36))
        msg_bytes = speed.to_bytes(2, byteorder="big")

        can_msg = can.Message(arbitration_id=CANFeedbackIdentifier.SPEED_SENSOR, data=msg_bytes)
        if CANFeedbackIdentifier.SPEED_SENSOR in self.__listeners:
            for listener in self.__listeners[CANFeedbackIdentifier.SPEED_SENSOR]:
                listener(can_msg)

        self.updating = False

    def __update_state(self) -> None:
        state = self.update_client.getCarState()
        self.__update(state.speed)

    def __run_loop(self) -> None:
        while True:
            car_controls = airsim.CarControls()
            car_controls.steering = self.steering
            car_controls.throttle = self.throttle
            car_controls.is_manual_gear = self.gear == Gear.REVERSE
            if car_controls.is_manual_gear:
                car_controls.manual_gear = -1

            car_controls.brake = self.brake
            self.update_client.setCarControls(car_controls)

            self.__update_state()
            time.sleep(1/40)
