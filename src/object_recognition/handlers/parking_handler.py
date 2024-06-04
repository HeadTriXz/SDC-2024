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

        self.controller.stop()
        self.controller.lane_assist.stop()
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
    
    def wait_for_wall(self) -> None:
        while self.__lidar.free_range(265, 275, 2500):
            time.sleep(0.1)

        self.wait_for_opening()

    def wait_for_opening(self) -> None:
        amount = 0
        while self.__lidar.free_range(265, 290, 3000):
            time.sleep(0.1)
            if amount == 3:
                break

            amount += 1

        if amount < 3:
            time.sleep(0.1)
            return self.wait_for_opening()

        return self.wait_for_steering_point()

    def wait_for_steering_point(self) -> None:
        """Wait for the steering point to be reached."""
        while True:
            time.sleep(0.1)
            if not self.__lidar.free_range(282, 290, 2000):
                a_side = self.__lidar.find_obstacle_distance(265, 280)
                c_side = self.__lidar.find_rightmost_point(280, 335, 800, 3000)
                if a_side == 0 or c_side == 0 or c_side < a_side:
                    continue

                b_side = math.sqrt((c_side ** 2) - (a_side ** 2))
                if b_side >= 160:
                    self.__speed_controller.toggle()
                    self.__can_controller.set_brake(100)
                    time.sleep(2)
                    self.__can_controller.set_brake(0)
                    self.__can_controller.set_throttle(20, Gear.REVERSE)
                    time.sleep(1)
                    self.__can_controller.set_steering(config["parking"]["steering_angle"])
                    self.drive_into_spot()
                    continue

    def drive_into_spot(self):
        """Drive into the parking spot."""
        print("spot")
        counter = 0

        while True:
            time.sleep(0.1)
            corner_angle = self.__lidar.find_nearest_angle(250, 300)
            corner = self.__lidar.scan_data[corner_angle]

            wall_1 = self.__lidar.find_lowest_index(200, corner_angle, corner, 8000)
            wall_2 = self.__lidar.find_highest_index(corner_angle, 320, corner, 8000)
            indices = np.where(self.__lidar.scan_data[corner_angle:320] == np.inf)[0]
            wall_1_distance = self.__lidar.scan_data[wall_1]
            wall_2_distance = self.__lidar.scan_data[wall_2]

            if len(indices) > 0:
                wall_2 = corner_angle + indices[0]

            if wall_1 == 0 or wall_2 == 0:
                continue
            print(corner_angle - wall_1)

            if corner_angle - wall_1 < 28 and wall_2 - corner_angle > 4 and (wall_2_distance - corner) > 50 and (
                    wall_1_distance - corner) > 50:
                if counter == 3:
                    self.__can_controller.set_steering(-config["parking"]["steering_angle"])
                    return self.wait_to_stop()
                counter += 1
            time.sleep(0.1)

    def wait_to_stop(self):
        """Wait till the go-kart almost crosses the line."""
        counter = 0
        while True:
            time.sleep(0.1)
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

                    logging.error("Cannot see the left wall, failed parking.")
                    return

                counter += 1
            time.sleep(0.1)

    def forward_creep(self, reverse: bool):
        """werk dan"""
        counter = 0
        while True:
            time.sleep(0.1)

            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(160, 310, 300, 9000)
            self.__can_controller.set_throttle(20, Gear.DRIVE)

            if right_wall == 0:
                continue
            deviation = (right_wall + left_wall) / 2 - 180

            if deviation > 2:
                self.__can_controller.set_steering(config["parking"]["steering_angle"])
            else:
                self.__can_controller.set_steering(-config["parking"]["steering_angle"])

            if -15 < deviation < 15:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                phase = 8
                continue

            if not self.__lidar.free_range(160, 220, 800) and reverse:
                self.reverse_creep(reverse)
                counter += 1
                continue

            if self.__lidar.free_range(160, 220, 2500):
                reverse = False
            self.__can_controller.set_brake(0)
            self.__can_controller.set_throttle(20, Gear.DRIVE)
            if self.__lidar.free_range(160, 220, 500):
                reverse = True

    def reverse_creep(self, reverse: bool):
        counter = 0
        while True:
            time.sleep(0.1)
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
                self.__can_controller.set_steering(config["parking"]["steering_angle"])
            else:
                self.__can_controller.set_steering(-config["parking"]["steering_angle"])

            if -15 < deviation < 15:
                self.__can_controller.set_brake(100)
                self.__can_controller.set_throttle(0, Gear.NEUTRAL)
                if self.__lidar.free_range(160, 250, 800):
                    self.forward_creep(reverse)
                return

            if not reverse:
                self.forward_creep(reverse)
            elif counter > 3:
                reverse = True

            if self.__lidar.free_range(120, 240, 2000):
                reverse = False
                counter = 0
                self.forward_creep(reverse)
            else:
                self.__can_controller.set_throttle(20, Gear.REVERSE)
            time.sleep(0.1)
