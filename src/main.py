from os import system
import can
from kart_control.can_controller import CANController
from kart_control.new_controller.__init__ import Controller
from kart_control.new_controller.basic_controller_driving import BasicControllerDriving
from procedures.brake_calibration import BrakeCalibrationProcedure

import logging

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    #try:
    system("ip link set can0 type can bitrate 500000")
    system("ip link set can0 up")
        #can_bus = can.Bus(interface="socketcan", channel="can0", bitrate=500000)
    #except OSError:
    can_bus = can.interface.Bus("can0", interface="virtual", bitrate=500000)

    controller = Controller()
    controller.start()
    driving = CANController(can_bus)
    controller_driving = BasicControllerDriving(driving, controller)
    controller_driving.start()
    procedure = BrakeCalibrationProcedure(driving, controller)
    while True:
        pass