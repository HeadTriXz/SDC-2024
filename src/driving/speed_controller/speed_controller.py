import can
import logging
import threading
import time

from src.config import config
from src.constants import CANFeedbackIdentifier, Gear
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController, SpeedControllerState
from src.utils.decorators import check_if


class SpeedController(ISpeedController):
    """A controller for the speed of the go-kart.

    Attributes
    ----------
        current_speed (float): The current speed of the go-kart.
        enabled (bool): Whether the speed controller is enabled.
        gear (Gear): The gear of the go-kart.
        max_speed (int): The maximum speed of the go-kart.
        target_speed (int): The target speed of the go-kart.
        state (SpeedControllerState): The state of the speed controller.

    """

    current_speed: float = 0
    enabled: bool = False

    __gear: Gear = Gear.NEUTRAL
    __can: ICANController
    __max_speed: int = 0
    __target_speed: int = 0
    __thread: threading.Thread
    __state: SpeedControllerState = SpeedControllerState.STOPPED

    def __init__(self, can_bus: ICANController) -> None:
        """Initialize the speed controller.

        :param can_bus: The CAN controller to use.
        """
        self.__can = can_bus
        self.logger = logging.getLogger(__name__)
        self.max_speed = config.kart.speed_modes.selected

        # FIXME: remove telemetry
        self.__thread = threading.Thread(target=self.__debug, daemon=True)

    @property
    def gear(self) -> Gear:
        """The gear of the go-kart."""
        return self.__gear

    @gear.setter
    def gear(self, gear: Gear) -> None:
        """Set the gear of the go-kart."""
        self.__gear = gear
        self.__adjust_speed()

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
    def can_controller(self) -> ICANController:
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

    def get_braking_distance(self) -> float:
        """Get the braking distance of the go-kart.

        :return: The braking distance in meters.
        """
        return (self.current_speed**2) / (250 * config.dynamic_speed.friction_coefficient)

    def start(self) -> None:
        """Start the speed controller."""
        self.__can.add_listener(CANFeedbackIdentifier.SPEED_SENSOR, self.__update_speed)
        if config.telemetry.enabled:
            self.__thread.start()

    def toggle(self) -> None:
        """Toggle the speed controller."""
        self.enabled = not self.enabled
        if not self.enabled:
            self.__can.set_throttle(0, Gear.NEUTRAL)
            self.__can.set_brake(config.kart.braking.max_force)

    def __debug(self) -> None:
        """Print debug information."""
        while True:
            logging.info(
                "Current: %s | Target: %s | Max: %s | State: %s",
                self.current_speed,
                self.__target_speed,
                self.__max_speed,
                self.state.name
            )
            time.sleep(1)

    @check_if("enabled")
    def __adjust_speed(self) -> None:
        """Adjust the speed of the kart."""
        if self.__state == SpeedControllerState.STOPPED:
            self.__can.set_throttle(0, Gear.NEUTRAL)
            self.__can.set_brake(config.kart.braking.max_force)
            return

        self.__can.set_throttle(self.__get_target_percentage(), self.__gear)
        self.__can.set_brake(
            0
            if self.current_speed <= (self.__target_speed + config.kart.braking.margin)
            else config.kart.braking.min_force
        )

    def __get_target_percentage(self) -> int:
        """Get the target percentage of the throttle to apply."""
        if self.__target_speed == 0:
            return 0

        if self.__target_speed >= config.kart.speed_modes.selected:
            return 100

        return int((self.__target_speed / config.kart.speed_modes.selected) * 100)

    def __update_speed(self, message: can.Message) -> None:
        """Update the speed of the go-kart."""
        value = int.from_bytes(message.data[:2], byteorder="big")

        self.current_speed = value / 10
        self.__adjust_speed()
