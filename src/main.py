from os import system

import can

from new_controller.BasicControllerDriving import BasicControllerDriving
from new_controller.CANController import CANController
from new_controller.controller import Controller
from procedures.brake_calibration import BrakeCalibrationProcedure

if __name__ == "__main__":
    system("ip link set can0 type can bitrate 500000")
    system("ip link set can0 up")

    can_bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
    # can_bus = can.interface.Bus("can0", interface="virtual", bitrate=500000)

    controller = Controller()
    controller.start()

    driving = CANController(can_bus)

    controller_driving = BasicControllerDriving(driving, controller)
    controller_driving.start()

    procedure = BrakeCalibrationProcedure(driving, controller)

    while True:
        pass
