import math
import numpy as np
import time

from ultralytics.engine.results import Boxes

from src.config import config
from src.constants import Gear, Label
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.object_recognition.object_controller import ObjectController
from src.utils.lidar.lidar import Lidar


class ParkingHandler(BaseObjectHandler):
    """A handler for parking spaces."""

    def __init__(self, controller: ObjectController, lidar: Lidar) -> None:
        """Initializes the parking handler.

        :param controller: The object controller.
        :param lidar: The lidar sensor.
        """
        super().__init__(controller, [Label.PARKING_SPACE])
        self.__lidar = lidar
        self.__speed_controller = controller.speed_controller
        self.__can_controller = controller.speed_controller.can_controller

    def handle(self, predictions: Boxes) -> None:
        """Check if there is a parking space available. If so, set the state to parking.

        :param predictions: The detected parking spaces.
        """
        if self.is_stopped_by_other():
            return

        if not self.__any_within_distance(predictions):
            return

        self.start_parking(0)

    def __any_within_distance(self, predictions: Boxes) -> bool:
        """Check if any parking space is within the distance threshold.

        :param predictions: The detected parking spaces.
        :return: Whether any parking space is within the distance threshold.
        """
        for bbox in predictions.xyxy:
            if not self.__is_on_my_right(bbox, predictions.orig_shape):
                continue

            if self.__is_within_distance(bbox, predictions.orig_shape):
                return True

        return False

    def __is_on_my_right(self, bbox: np.ndarray, shape: tuple[int, int]) -> bool:
        """Check if the parking space is on the right side of the go-kart.

        :param bbox: The bounding box of the parking space.
        :param shape: The shape of the image (height, width).
        :return: Whether the parking space is on the right side of the go-kart.
        """
        return bbox[0] > shape[1] // 2

    def __is_within_distance(self, bbox: np.ndarray, shape: tuple[int, int]) -> bool:
        """Check if the parking space is within the distance threshold.

        :param bbox: The bounding box of the parking space.
        :param shape: The shape of the image (height, width).
        :return: Whether the parking space is within the distance threshold.
        """
        x, y = (bbox[0] + bbox[2]) // 2, bbox[3]
        distance = self.controller.calibration.get_distance_to_y(x, y, shape[::-1])

        return distance < config["parking"]["min_distance"]

    def start_parking(self, phase: int) -> None:
        """Start the parking.

        :param phase: The phase of the parking.
        """
        self.controller.stop()
        self.controller.lane_assist.stop()
        self.__speed_controller.target_speed = 3

        time.sleep(1)
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
            if not self.__lidar.free_range(282, 290, 200):
                a_side = self.__lidar.find_obstacle_distance(265, 280)
                c_side = self.__lidar.find_rightmost_point(280, 335, 80, 300)
                if a_side == 0 or c_side == 0 or c_side < a_side:
                    continue
                c_side = int(c_side)
                a_side = int(a_side)
                b_side = math.sqrt((c_side ** 2) - (a_side ** 2))
                if b_side >= 160:
                    self.__speed_controller.toggle()
                    self.controller.lane_assist.toggle()
                    self.__can_controller.set_brake(100)
                    time.sleep(2)
                    self.__can_controller.set_brake(0)
                    self.__can_controller.set_throttle(20, Gear.REVERSE)
                    time.sleep(1)
                    self.__can_controller.set_steering(1.25)
                    phase = 4
                    continue

        counter = 0
        while phase == 4:
            corner_angle = self.__lidar.find_nearest_angle(250, 300)
            corner = self.__lidar.points[corner_angle]

            wall_1 = self.__lidar.find_lowest_index(200, corner_angle, corner, 800)
            wall_2 = self.__lidar.find_highest_index(corner_angle, 320, corner, 800)
            indices = np.where(self.__lidar.points[corner_angle:320] == np.inf)[0]
            wall_1_distance = self.__lidar.points[wall_1]
            wall_2_distance = self.__lidar.points[wall_2]

            if len(indices) > 0:
                wall_2 = corner_angle + indices[0]

            if wall_1 == 0 or wall_2 == 0:
                continue

            if corner_angle - wall_1 < 45 and wall_2 - corner_angle > 5 and (wall_2_distance - corner) > 50 and (
                    wall_1_distance - corner) > 50:
                if counter == 3:
                    self.__can_controller.set_steering(-1.25)
                    phase = 5
                counter += 1
            time.sleep(0.1)

        while phase == 5:
            lowest_angle = self.__lidar.find_lowest_index(0, 150, 30, 900)
            highest_angle = self.__lidar.find_highest_index(180, 320, 30, 900)
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

            left_wall = self.__lidar.find_lowest_index(130, 250, 30, 900)
            right_wall = self.__lidar.find_highest_index(160, 280, 30, 900)
            self.__can_controller.set_throttle(20, Gear.DRIVE)

            if right_wall == 0:
                continue
            deviation = (right_wall + left_wall) / 2 - 180
            if deviation > 2:
                self.__can_controller.set_steering(1.25)
            else:
                self.__can_controller.set_steering(-1.25)

            if -23 < deviation < 23:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8
                continue

            if not self.__lidar.free_range(160, 220, 80) and reverse:
                phase = 7
                continue
            if self.__lidar.free_range(160, 220, 250):
                reverse = False
            self.__can_controller.set_brake(0)
            self.__can_controller.set_throttle(20, Gear.DRIVE)
            if self.__lidar.free_range(160, 220, 50):
                reverse = True

        while phase == 7:
            left_wall = self.__lidar.find_lowest_index(130, 240, 30, 900)
            right_wall = self.__lidar.find_highest_index(150, 260, 30, 900)
            deviation = ((right_wall - 180) + (180 - left_wall)) / 2

            lowest_angle = self.__lidar.find_lowest_index(30, 150, 70, 900)
            highest_angle = self.__lidar.find_highest_index(180, 320, 70, 900)
            angle = highest_angle - lowest_angle

            if angle < 230 and deviation > 40:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.DRIVE)
                self.__can_controller.set_steering(0)
                reverse = False
                self.start_parking(6)

            if deviation < 0:
                self.start_parking(6)
                self.__can_controller.set_steering(1.25)
            else:
                self.__can_controller.set_steering(-1.25)

            if -25 < deviation < 25:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                if self.__lidar.free_range(160, 250, 80):
                    self.start_parking(6)

            if not reverse:
                self.start_parking(6)

            if self.__lidar.free_range(160, 250, 250):
                reverse = False
                self.start_parking(6)
            else:
                self.__can_controller.set_throttle(20, Gear.REVERSE)
            time.sleep(0.1)
