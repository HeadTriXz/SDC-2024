import airsim

from src.driving.speed_controller import SpeedController, SpeedControllerState
from src.object_recognition.handlers.parking_handler import ParkingManoeuvre
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
    parking = ParkingManoeuvre(lidar, speed_controller, can_controller)

    lidar.start()
    can_controller.start()
    speed_controller.start()

    parking.park()
