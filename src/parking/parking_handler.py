import math
import numpy as np
import time

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

    def start_parking(self, phase) -> None:
        """Start the parking."""
        time.sleep(1)
        self.__speed_controller.target_speed = 3
        while phase == 0:
            if not self.__lidar.free_range(265, 275, 250):
                phase = 1
                continue
            time.sleep(0.1)
        amount = 0

        while phase == 1:
            if self.__lidar.free_range(265, 290, 300):
                amount += 1
                if amount == 3:
                    phase = 2
                continue
            time.sleep(0.1)

        while phase == 2:
            time.sleep(0.1)
            print(3)
            if not self.__lidar.free_range(282, 290, 200):
                a_side = self.__lidar.find_obstacle_distance(265, 280)
                c_side = self.__lidar.find_rightmost_point(280, 335, 80, 300)
                if a_side == 0 or c_side == 0 or c_side < a_side:
                    continue
                c_side = int(c_side)
                a_side = int(a_side)
                b_side = math.sqrt((c_side ** 2) - (a_side ** 2))
                if b_side >= 160:
                    print("Parking")
                    self.__speed_controller.toggle()
                    self.__can_controller.set_brake(100)
                    time.sleep(2)
                    self.__can_controller.set_brake(0)
                    self.__can_controller.set_throttle(20, Gear.REVERSE)
                    time.sleep(1)
                    self.__can_controller.set_steering(0.8)
                    phase = 4
                    continue

        counter = 0
        while phase == 4:
            if self.__lidar.free_range(200,240, 400):
                if counter == 3:
                    self.__can_controller.set_steering(-0.8)
                    phase = 5
                    continue
                counter += 1

            print(4)
            time.sleep(0.1)
            corner_angle = self.__lidar.find_nearest_angle(250, 300)
            corner = self.__lidar.points[corner_angle]

            wall_1 = self.__lidar.find_lowest_index(200, corner_angle, corner + 20, 600)
            wall_2 = 0
            indices = np.where(self.__lidar.points[corner_angle:320] == np.inf)[0]

            if len(indices) > 0:
                wall_2 = corner_angle + indices[0]

            if wall_1 == 0 or wall_2 == 0:
                continue

            if corner_angle - wall_1 < 25 and wall_2 - corner_angle > 5 and (self.__lidar.points[wall_2] - self.__lidar.points[corner_angle]) >100 and (self.__lidar.points[wall_1]- self.__lidar.points[corner_angle]) > 100:
            #if corner_angle > 260:
                if counter == 3:
                    self.__can_controller.set_steering(-0.8)
                    phase = 5
                counter += 1

        while phase == 5:
            lowest_angle = self.__lidar.find_lowest_index(0, 150, 30, 900)
            highest_angle = self.__lidar.find_highest_index(180, 320, 30, 900)
            angle = highest_angle - lowest_angle
            if angle < 230:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                self.__can_controller.set_steering(0)
                phase = 6
            time.sleep(0.1)

        while phase == 6:
            print(6)
            left_wall = self.__lidar.find_lowest_index(130, 200, 30, 900)
            right_wall = self.__lidar.find_highest_index(160, 230, 30, 900)
            if right_wall == 0:
                continue
            deviation = ((right_wall - 180) + (180 - left_wall)) / 2
            if deviation > 20:
                phase = 7
            print(deviation)
            if -20 < deviation < 20:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8
            self.__can_controller.set_steering(0.8)
            self.__can_controller.set_brake(0)
            self.__can_controller.set_throttle(20, Gear.DRIVE)
            if self.__lidar.find_obstacle_distance(180, 230) < 100:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 7

            if self.__lidar.free_range(185,240, 400):
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8
            time.sleep(0.1)

        while phase == 7:
            print(7)
            left_wall = self.__lidar.find_lowest_index(130, 200, 30, 900)
            right_wall = self.__lidar.find_highest_index(160, 230, 30, 900)
            if left_wall == 0:
                continue
            deviation = ((right_wall - 180) + (180 - left_wall)) / 2
            if deviation < 20:
                self.start_parking(6)
            if -20 < deviation < 20:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8

            self.__can_controller.set_steering(-0.8)
            self.__can_controller.set_brake(0)
            self.__can_controller.set_throttle(20, Gear.REVERSE)

            if self.__lidar.free_range(220, 250, 300):
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8

            # if self.__lidar.free_range(160,240, 300):
            #     self.__can_controller.set_brake(100)
            #     self.start_parking(6)
            # time.sleep(0.1)

