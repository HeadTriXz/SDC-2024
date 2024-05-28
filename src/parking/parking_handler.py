import time
import math

from src.constants import Gear
from src.driving.speed_controller import ISpeedController
from src.simulation.sim_lidar import SimLidar


class ParkingHandler:
    """A class that handles the parking."""

    def __init__(self, lidar: SimLidar, speed_controller: ISpeedController) -> None:
        """Create a parking handler."""
        self.__lidar = lidar
        self.__speed_controller = speed_controller
        self.__can_controller = speed_controller.can_controller

    def start_parking(self) -> None:
        """Start the parking."""
        time.sleep(3)
        self.__speed_controller.target_speed = 3.0
        distance = 7
        phase = 0
        initial_distance = 0
        distance_wall = 0
        wall_detected = False
        counter = 0
        # self.__can_controller.set_steering(1.25)
        while phase == 0:
            self.__can_controller.target_speed = 1
            if not self.__lidar.free_range(265, 275, 250):
                distance_wall = self.__lidar.find_obstacle_distance(255, 285)
                print(self.__lidar.points[260:280])
                phase = 1
                continue
            print("uh")
            time.sleep(0.1)

        while phase == 1:
            if self.__lidar.free_range(270, 275, 300):
                phase = 2
                continue
            print("waiting for parking spot detection")
            print(self.__lidar.points[260:280])
            time.sleep(0.1)
        while phase == 2:
            if not self.__lidar.free_range(282, 290, 200):
                Aside = self.__lidar.find_obstacle_distance(265, 280)
                Cside = self.__lidar.find_highest_index(280, 320, 90, 300)
                print(Aside, Cside)
                Cside = int(Cside)
                Aside = int(Aside)
                Bside = math.sqrt((Cside ** 2)- (Aside ** 2))
                if Bside >= 160:
                    print('kut kint')
                    self.__can_controller.set_brake(69)
                    self.__can_controller.set_steering(1.25)
                    time.sleep(1)
                    # self.__speed_controller.gear
                    # ake(0)
                    continue
        while phase == 3:
            current_distance = self.__lidar.find_obstacle_distance(250, 290)
            if current_distance < distance:
                distance = current_distance
                counter = 0
            else:
                if counter == 2:
                    self.__can_controller.set_steering(-1.25)
                    phase = 4
                counter += 1
        while phase == 4:
            if phase == 4:
                if not self.__lidar.free_range(240, 245, 100):
                    self.__can_controller.set_brake(100)
