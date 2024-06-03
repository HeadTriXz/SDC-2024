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

    def start_parking(self, phase: int) -> None:
        """Start the parking.

        :param phase: The phase of the parking.
        """
        time.sleep(1)
        self.__speed_controller.target_speed = 3
        while phase == 0:
            if not self.__lidar.free_range(265, 275, 2500):
                phase = 1
                continue
            time.sleep(0.1)
        amount = 0

        while phase == 1:
            if self.__lidar.free_range(265, 290, 3000):
                amount += 1
                if amount == 3:
                    phase = 2
                continue
            time.sleep(0.1)

        while phase == 2:
            time.sleep(0.1)
            if not self.__lidar.free_range(282, 290, 2000):
                a_side = self.__lidar.find_obstacle_distance(265, 280)
                c_side = self.__lidar.find_rightmost_point(280, 335, 800, 3000)
                if a_side == 0 or c_side == 0 or c_side < a_side:
                    continue
                c_side = int(c_side)
                a_side = int(a_side)
                b_side = math.sqrt((c_side ** 2) - (a_side ** 2))
                if b_side >= 160:
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

            corner_angle = self.__lidar.find_nearest_angle(250, 300)
            corner = self.__lidar.points[corner_angle]

            wall_1 = self.__lidar.find_lowest_index(200, corner_angle, corner, 8000)
            wall_2 = self.__lidar.find_highest_index(corner_angle, 320, corner, 8000)
            indices = np.where(self.__lidar.points[corner_angle:320] == np.inf)[0]
            wall_1_distance = self.__lidar.points[wall_1]
            wall_2_distance = self.__lidar.points[wall_2]

            if len(indices) > 0:
                wall_2 = corner_angle + indices[0]

            if wall_1 == 0 or wall_2 == 0:
                continue

            if corner_angle - wall_1 < 20 and wall_2 - corner_angle > 4 and (wall_2_distance - corner) > 50 and (
                    wall_1_distance - corner) > 50:
                if counter == 3:
                    self.__can_controller.set_steering(-0.8)
                    phase = 5
                counter += 1
            time.sleep(0.1)

        while phase == 5:

            lowest_angle = self.__lidar.find_lowest_index(0, 150, 300, 9000)
            highest_angle = self.__lidar.find_highest_index(180, 320, 300, 9000)
            angle = highest_angle - lowest_angle

            if angle < 230:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                self.__can_controller.set_steering(0)
                phase = 6
            if lowest_angle == 0:
                if counter == 3:
                    self.__can_controller.set_brake(100)
                    phase = 3
                    continue
                counter += 1
            time.sleep(0.1)
        reverse = False

        while phase == 6:
            time.sleep(0.1)

            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(160, 310, 300, 9000)
            self.__can_controller.set_throttle(20, Gear.DRIVE)

            if right_wall == 0:
                continue
            deviation = (right_wall + left_wall) / 2 - 180

            if deviation > 2:
                self.__can_controller.set_steering(0.8)
            else:
                self.__can_controller.set_steering(-0.8)

            if -15 < deviation < 15:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8
                continue

            if not self.__lidar.free_range(160, 220, 800) and reverse:
                phase = 7
                counter +=1
                continue

            if self.__lidar.free_range(160, 220, 2500):
                reverse = False
            self.__can_controller.set_brake(0)
            self.__can_controller.set_throttle(20, Gear.DRIVE)
            if self.__lidar.free_range(160, 220, 500):
                reverse = True

        while phase == 7:
            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(150, 310, 300, 9000)
            deviation = (right_wall + left_wall) / 2 - 180

            lowest_angle = self.__lidar.find_lowest_index(30, 150, 700, 9000)
            highest_angle = self.__lidar.find_highest_index(180, 320, 700, 9000)
            angle = highest_angle - lowest_angle

            if angle < 233 and deviation > 40:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.DRIVE)
                self.__can_controller.set_steering(0)
                reverse = False
                self.start_parking(6)

            if deviation < 0:
                self.start_parking(6)
                self.__can_controller.set_steering(0.8)
            else:
                self.__can_controller.set_steering(-0.8)

            if -15 < deviation < 15:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                if self.__lidar.free_range(160, 250, 800):
                    self.start_parking(8)

            if not reverse:
                self.start_parking(6)
            elif counter > 3:
                reverse = True

            if self.__lidar.free_range(120, 240, 2000):
                reverse = False
                counter = 0
                self.start_parking(6)

            else:
                self.__can_controller.set_throttle(20, Gear.REVERSE)
            time.sleep(0.1)
