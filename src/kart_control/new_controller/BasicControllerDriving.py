import threading

from common.constants import Gear
from kart_control.can_controller import CANController
from kart_control.new_controller import (
    Controller,
    ControllerAxis,
    ControllerButton,
    EventType,
)


class BasicControllerDriving:
    direction = Gear.NEUTRAL

    def __init__(self, can_controller: CANController, controller: Controller) -> None:
        """Driving using a controller.

        the controller scheme is as follows:
        - left joystick: steering
        - right trigger: throttle
        - left trigger: brake
        - X: reverse
        - Y: neutral
        - B: forward
        - A: arm/start driving
        """
        self.can_controller = can_controller
        self.controller = controller
        self.__run_thread = threading.Thread(target=self.__run, daemon=True)

    def start(self) -> None:
        """Start driving using controller.

        ## sequence to start driving:
        1. wait until vibration.
        2. hold A to arm
        3. wait until vibration
        4. drive

        ## controlls:
        - left joystick: steering
        - right trigger: throttle
        - left trigger: brake
        - X: reverse
        - Y: neutral
        - B: forward
        """
        # start __run on a new thread
        self.__run()

    def __run(self) -> None:
        print("starting")

        # set the throttle to 0 and apply full braking
        self.can_controller.set_throttle(0, Gear.NEUTRAL)
        self.can_controller.set_brake(100)  # 100% braking

        # register callbacks for the buttons adn axis
        self.controller.add_listener(EventType.LONG_PRESS, ControllerButton.A, self.__ready)

        # vibrate the controller after 1 sec
        self.controller.vibrate(500)
        print("shake it off")

    def __ready(self, _event: EventType, _button: ControllerButton) -> None:
        print("__ready")
        self.controller.vibrate(1000)

        # gears
        self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.X, self.__set_reverse)
        self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.Y, self.__set_neutral)
        self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.B, self.__set_forward)
        # throttle and brake
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.RT, self.__set_throttle)
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LT, self.__set_brake)
        # steering
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LS_X, self.__set_steering)

    def __set_reverse(self, _type: TypeError, _button: ControllerButton) -> None:
        self.direction = Gear.REVERSE

    def __set_neutral(self, _type: TypeError, _button: ControllerButton) -> None:
        self.direction = Gear.NEUTRAL

    def __set_forward(self, _type: TypeError, _button: ControllerButton) -> None:
        self.direction = Gear.DRIVE

    def __set_throttle(self, _event: EventType, _button: ControllerButton, val: float) -> None:
        self.can_controller.set_throttle(int(val * 100), self.direction)

    def __set_brake(self, _event: EventType, _button: ControllerButton, val: float) -> None:
        self.can_controller.set_brake(int(val * 100))

    def __set_steering(self, _event: EventType, _button: ControllerButton, val: float) -> None:
        if abs(val) > 0.1:
            self.can_controller.set_steering(val)
