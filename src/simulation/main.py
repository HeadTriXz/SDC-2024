import airsim

from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.parking.parking_handler import ParkingHandler
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
    parking_handler = ParkingHandler(lidar, speed_controller)

    lidar.start()
    can_controller.start()
    speed_controller.start()

    parking_handler.start_parking(0)
