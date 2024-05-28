import airsim
import cv2
import numpy as np

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

    parking_handler.start_parking()

    # while True:
    #     print("Getting lidar data")
    #     # plot the lidar data
    #     lidar_data = lidar.points
    #
    #     # convert the points to cartesian coordinates
    #     x = lidar_data * np.cos(np.deg2rad(np.arange(360)))
    #     y = lidar_data * np.sin(np.deg2rad(np.arange(360)))
    #
    #     img = np.zeros((512, 512), np.uint8)
    #
    #     # draw the points on the image. the farter away the point is, the darker it is
    #     for i in range(360):
    #         # draw the dots
    #         if lidar_data[i] != np.inf:
    #             x_pos = x[i] + 256
    #             y_pos = y[i] + 256
    #
    #             # check if they are not inf
    #             if x_pos < 0 or x_pos >= 512 or y_pos < 0 or y_pos >= 512:
    #                 continue
    #
    #             cv2.circle(img, (int(x_pos), int(y_pos)), 3, int(255 - lidar_data[i] / 600 * 255), -1)
    #
    #     cv2.imshow("Lidar", img)
    #     if cv2.waitKey(1) & 0xFF == ord("q"):
    #         break
