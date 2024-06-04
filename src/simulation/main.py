import airsim
import numpy as np

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.lane_assist.stop_line_assist import StopLineAssist
from src.object_recognition.handlers.parking_handler import ParkingHandler
from src.object_recognition.object_controller import ObjectController
from src.simulation.can_controller import SimCanController
from src.simulation.sim_lidar import SimLidar


def start_simulator() -> None:
    """Run the simulator."""
    # Start the client.
    client = airsim.CarClient()
    client.confirmConnection()

    can_controller = SimCanController(True)
    speed_controller = SpeedController(can_controller)
    speed_controller.state = SpeedControllerState.DRIVING
    speed_controller.toggle()

    lidar = SimLidar(client)
    calibration = CalibrationData.load(config["calibration"]["calibration_file"])

    stop_line_assist = StopLineAssist(speed_controller, calibration)
    lane_assist = LaneAssist(
        lambda: np.zeros((1, 1)),
        stop_line_assist,
        speed_controller,
        None,
        calibration
    )
    object_controller = ObjectController(calibration, lane_assist, speed_controller)
    parking_handler = ParkingHandler(object_controller, lidar)

    lidar.start()
    can_controller.start()
    speed_controller.start()

    speed_controller.target_speed = 5
    parking_handler.wait_for_wall()
