from constants import Gear
from driving.can_controller.can_controller_interface import ICANController
from driving.speed_controller import SpeedControllerState
from driving.speed_controller.speed_controller_interface import ISpeedController
from test_lane_assist.mocks.mock_can_controller import MockCanController


class MockSpeedController(ISpeedController):
    """A mock speed controller for testing purposes."""

    __gear: Gear = Gear.NEUTRAL
    __state: SpeedControllerState = SpeedControllerState.STOPPED
    __can: ICANController = MockCanController()
    __max_speed: int = 0
    __target_speed: int = 0

    @property
    def gear(self) -> Gear:
        """The gear of the go-kart."""
        return self.__gear

    @gear.setter
    def gear(self, gear: Gear) -> None:
        """Set the gear of the go-kart."""
        self.__gear = gear

    @property
    def state(self) -> SpeedControllerState:
        """The state of the speed controller."""
        return self.__state

    @state.setter
    def state(self, state: SpeedControllerState) -> None:
        """Set the state of the speed controller."""
        self.__state = state

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
        self.__max_speed = speed

    @property
    def target_speed(self) -> int:
        """The target speed of the go-kart."""
        return self.__target_speed

    @target_speed.setter
    def target_speed(self, speed: int) -> None:
        """Set the target speed of the go-kart."""
        self.__target_speed = speed

    def start(self) -> None:
        """Start the speed controller."""
        pass
