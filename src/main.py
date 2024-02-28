import can

from new_controller.BasicControllerDriving import BasicControllerDriving
from new_controller.CANController import CANController
from new_controller.controller import Controller

if __name__ == "__main__":
    can_bus = can.interface.Bus("can0", interface="virtual", bitrate=500000)

    controller = Controller()
    controller.start()

    driving = CANController(can_bus)

    controller_driving = BasicControllerDriving(driving, controller)
    controller_driving.start()

    # procedure = BrakeCalibrationProcedure(driving, controller)

    while True:
        pass
