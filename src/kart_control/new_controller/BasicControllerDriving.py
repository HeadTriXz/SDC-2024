# ruff: noqa: T201

import threading
import time

import can

from kart_control.can_controller import CANController
from kart_control.new_controller.controller import (
    Controller,
    ControllerAxis,
    ControllerButton,
    EventType,
)


class BasicControllerDriving(CANController):
    """Class that will drive the kart with an xbox controller.

    this class will register all needed buttons on the controller.
    the controll scheme is as follows:
        - left joystick: steering
        - right trigger: throttle
        - left trigger: brake
        - X: reverse
        - Y: neutral
        - B: forward
        - A: arm/start driving


    Example:
    -------
    ```python
    from kartcontroll.new_contorller.controller import Controller
    from kartcontroll.new_contorller. import Controller
    import can

    if __name__ == "__main__":
        can_bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

        controller = Controller()
        controller.start()

        controller_driving = BasicControllerDriving(can_bus, controller)
        controller_driving.start()
    ```

    this piece of code will start the canbus, controller and controller driving.
    ater that you need to hold a and then you can drive.

    """

    direction = 0

    def __init__(self, can_bus: can.Bus, controller: Controller) -> None:
        """Driving using a controller.

        Parameters
        ----------
        :param can_bus can.Bus: the canbus that is used to controll the kart.
        :param controller Controller: the controller to register the buttons on.

        """
        super().__init__(can_bus)
        self.controller = controller

    def start(self) -> None:
        """Start driving using a controller."""
        # start __run on a new thread
        self.__run()

    def __run(self) -> None:
        """Run the actual driving script.

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
        print("starting")
        # start the controller
        self.controller_thread = threading.Thread(target=self.controller.start, daemon=True)
        self.controller_thread.start()

        # set the throttle to 0 and apply full braking
        self.set_throttle(0, 0)

        self.set_brake(100)  # 100% braking-+

        print("shit done")

        # vibrate the controller after 1 sec
        time.sleep(1)
        self.controller.vibrate(1000)
        print("shake it off")

    a_time: int

    def __ready(self, event: EventType, _button: ControllerButton) -> None:
        print("__ready")
        if event == EventType.BUTTON_DOWN:
            self.a_time = time.time()
            print("set time")
        else:
            print(self.a_time - time.time())
            if time.time() - self.a_time > 1.5:
                print("ready")
                self.controller.vibrate(1000)

                # register callbacks for the buttons adn axis
                self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.A, self.__ready)
                self.controller.add_listener(EventType.BUTTON_UP, ControllerButton.A, self.__ready)

                # gears
                self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.X, self.__set_reverse)
                self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.Y, self.__set_neutral)
                self.controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.B, self.__set_forward)
                # throttle and brake
                self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.RT, self.__set_throttle)
                self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LT, self.__set_brake)
                # steering
                self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LS_X, self.__set_steering)

    def __set_reverse(self, _event: EventType, _button: ControllerButton) -> None:
        if self.ready:
            self.direction = 2

    def __set_neutral(self, _event: EventType, _button: ControllerButton) -> None:
        if self.ready:
            self.direction = 0

    def __set_forward(self, _event: EventType, _button: ControllerButton) -> None:
        if not self.ready:
            return
        self.direction = 1

    def __set_throttle(self, _event: EventType, button_val: tuple[ControllerButton, float]) -> None:
        if self.ready:
            self.set_throttle(int(button_val[1] * 100), self.direction)

    def __set_brake(self, _event: EventType, value: tuple[ControllerButton, float]) -> None:
        if self.ready:
            self.set_brake(int(value[1] * 100))

    def __set_steering(self, _event: EventType, value: tuple[ControllerButton, float]) -> None:
        if self.ready and abs(value[1]) > 0.1:
            self.set_steering(value[1])
