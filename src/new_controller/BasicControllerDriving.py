from src.new_controller.BasicDriving import BasicDriving
from src.new_controller.controller import Controller, EventType, ControllerButton, ControllerAxis

import threading
import time
import can

class BasicControllerDriving(BasicDriving):

    direction = 0

    def __init__(self, can_bus: can.Bus):
        super().__init__(can_bus)
        self.controller = Controller()

    def start(self):
        """
        Start driving using a controller
        :return:
        """

        # start __run on a new thread
        self.__run_thread = threading.Thread(target=self.__run)
        self.__run_thread.daemon = True
        self.__run_thread.start()

    def __run(self):
        """ Run the actual driving script.

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

        # start the controller
        self.controller.start()

        # register callbacks for the buttons adn axis
        self.controller.add_listener(EventType.LONG_PRESS, ControllerButton.A, self.__ready)
        # gears
        self.controller.add_listener(EventType.SHORT_PRESS, ControllerButton.X, self.__set_reverse)
        self.controller.add_listener(EventType.SHORT_PRESS, ControllerButton.Y, self.__set_neutral)
        self.controller.add_listener(EventType.SHORT_PRESS, ControllerButton.B, self.__set_forward)
        # throttle and brake
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.RT, self.__set_throttle)
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LT, self.__set_brake)
        # steering
        self.controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LS_X, self.__set_steering)

        # set the throttle to 0 and apply full braking
        self.set_throttle(0, 0)
        self.set_brake(100) # 100% braking

        # vibrate the controller after 1 sec
        time.sleep(1)
        self.controller.vibrate(1000)

    def __ready(self):
        self.ready = True
        self.controller.vibrate(1000)

    def __set_reverse(self):
        self.direction = 2

    def __set_neutral(self):
        self.direction = 0

    def __set_forward(self):
        self.direction = 1

    def __set_throttle(self, value):
        if self.ready:
            self.set_throttle(int(value * 100), self.direction)

    def __set_brake(self, value):
        if self.ready:
            self.set_brake(int(value * 100))

    def __set_steering(self, value):
        if self.ready and abs(value) > 0.1 :
            self.set_steering(value)





