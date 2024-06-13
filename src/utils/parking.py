import logging
import math
import numpy as np
import time

from typing import Callable

from src.config import config
from src.constants import Gear
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.utils.lidar import BaseLidar


class ParkingManoeuvre:
    """A parking manoeuvre that uses the lidar sensor to park the go-kart."""

    __can_controller: ICANController
    __lane_assist: LaneAssist
    __lengths: list[float]
    __lidar: BaseLidar
    __speed_controller: ISpeedController

    def __init__(self, lidar: BaseLidar, lane_assist: LaneAssist) -> None:
        """Initializes the parking manoeuvre.

        :param lidar: The lidar sensor.
        :param lane_assist: The lane assist system.
        """
        self.__lengths = []
        self.__lidar = lidar
        self.__lane_assist = lane_assist
        self.__speed_controller = lane_assist.speed_controller
        self.__can_controller = lane_assist.can_controller

    def park(self) -> None:
        """Park the go-kart."""
        self.__speed_controller.target_speed = config["parking"]["max_speed"]
        logging.info("Parking the go-kart.")

        self.__wait_until_wall()
        self.__wait_until_opening()
        self.__wait_until_wall(estimate_length=True)

        available_space = np.median(self.__lengths) / 1000
        logging.info("Estimated parking space length: %.2f meters.", available_space)

        if available_space < config["kart"]["dimensions"]["length"]:
            self.__speed_controller.state = SpeedControllerState.STOPPED

            logging.error("Parking space is too small.")
            return

        # Drive into the parking spot
        self.__wait_until(self.__should_start_steering)
        self.__lane_assist.stop()
        self.__start_steering()

        # Straighten the wheels when the go-kart is at a 45-degree angle
        self.__wait_until(self.__is_at_45_degree_angle)
        self.__straighten_wheels()

        # Stop backing up once the front of the go-kart is past the barrier, then start steering back
        self.__wait_until(self.__is_past_barrier)
        self.__start_steering_back()

        # Wait until the go-kart has parked, then stop driving
        self.__wait_until(self.__is_straight)
        self.__straighten_wheels()

        self.__wait_until(lambda: self.__is_centered(available_space))
        self.__speed_controller.toggle()

    def __angle_to_xy(self, angle: int) -> tuple[float, float]:
        """Convert an angle and distance to x, y coordinates.

        :param angle: The angle to convert.
        :return: The x, y coordinates.
        """
        radians = math.radians((angle + 90) % 360)
        distance = self.__lidar.scan_data[angle]

        x = distance * math.cos(radians)
        y = distance * math.sin(radians)

        return x, y

    def __estimate_length(self) -> None:
        """Estimate the length of the parking space."""
        angle_front = self.__lidar.find_nearest_angle(200, 250)
        angle_back = self.__lidar.find_nearest_angle(280, 320)
        if angle_front == -1 or angle_back == -1:
            return

        angle_diff = math.radians(angle_back - angle_front)

        dist_front = self.__lidar.scan_data[angle_front]
        dist_back = self.__lidar.scan_data[angle_back]

        if np.isinf(dist_front) or np.isinf(dist_back):
            return

        distance = math.sqrt(dist_front**2 + dist_back**2 - 2 * dist_front * dist_back * math.cos(angle_diff))
        self.__lengths.append(distance)

    def __is_at_45_degree_angle(self) -> bool:
        """Check if the go-kart is at a 45-degree angle.

        :return: Whether the go-kart is at a 45-degree angle.
        """
        outer_angle = self.__lidar.find_lowest_index(200, 270, 300, 5000)
        inner_angle = self.__lidar.find_highest_index(250, 320, 300, 5000)

        if outer_angle == -1 and inner_angle == -1:
            return False

        nearest_angle = self.__lidar.find_nearest_angle(200, 300)
        if nearest_angle == -1:
            return False

        nearest_point = self.__angle_to_xy(nearest_angle)
        angles: list[float] = []

        if outer_angle != -1:
            outer_point = self.__angle_to_xy(outer_angle)
            outer_wall_angle = self.__angle_from_points(outer_point, nearest_point)
            angles.append(outer_wall_angle)

        if inner_angle != -1:
            inner_point = self.__angle_to_xy(inner_angle)
            inner_wall_angle = self.__angle_from_points(nearest_point, inner_point)
            if inner_wall_angle >= 130:
                angles.append(inner_wall_angle - 90)

        if len(angles) == 0:
            return False

        return np.mean(angles) <= 45 + config["parking"]["angle_tolerance"]

    def __is_centered(self, available_space: float) -> bool:
        """Check if the go-kart is centered in the parking spot.

        :param available_space: The available space in the parking spot.
        :return: Whether the go-kart is centered in the parking spot.
        """
        frontal_distance = self.__lidar.find_obstacle_distance(175, 185)

        # Something went wrong, we need to stop the go-kart
        if np.isinf(frontal_distance):
            return True

        margin = (config["kart"]["dimensions"]["length"] / 2) - config["kart"]["lidar_offset"]
        margin -= self.__speed_controller.get_braking_distance()

        center = available_space / 2 * 1000

        return frontal_distance - margin * 1000 >= center

    def __is_past_barrier(self) -> bool:
        """Check if the go-kart is past the barrier.

        :return: Whether the go-kart is past the barrier.
        """
        car_x = config["kart"]["dimensions"]["width"] / 2 * 1000
        car_y = (-config["kart"]["lidar_offset"] - config["parking"]["min_barrier_distance"]) * 1000

        nearest_angle = self.__lidar.find_nearest_angle(200, 300)
        rightmost_angle = self.__lidar.find_highest_index(200, 300, 300, 3000)
        if nearest_angle == -1 or rightmost_angle == -1:
            return False

        nearest_point = self.__angle_to_xy(nearest_angle)
        rightmost_point = self.__angle_to_xy(rightmost_angle)

        wall_angle = self.__angle_from_points(nearest_point, rightmost_point)
        if wall_angle < 120:
            return False

        angle = self.__angle_from_points((car_x, car_y), rightmost_point)
        return angle >= wall_angle - config["parking"]["angle_tolerance"]

    def __is_straight(self) -> bool:
        """Check if the go-kart is straight in the parking spot.

        :return: Whether the go-kart is straight in the parking spot.
        """
        nearest_angle = self.__lidar.find_nearest_angle(150, 210)
        rightmost_angle = self.__lidar.find_highest_index(150, 210, 300, 5000)

        if nearest_angle == -1 or rightmost_angle == -1 or nearest_angle == rightmost_angle:
            return False

        leftmost_point = self.__angle_to_xy(nearest_angle)
        rightmost_point = self.__angle_to_xy(rightmost_angle)

        angle = self.__angle_from_points(leftmost_point, rightmost_point)
        min_angle = 180 - config["parking"]["straight_angle_margin"]
        max_angle = 180 + config["parking"]["straight_angle_margin"]

        return min_angle <= angle <= max_angle

    def __should_start_steering(self) -> bool:
        """Keep driving until we can start steering into the parking spot.

        :return: Whether we can start steering into the parking spot.
        """
        nearest_angle = self.__lidar.find_nearest_angle(265, 275)
        rightmost_angle = self.__lidar.find_highest_index(275, 320, 1000, 3000)

        if nearest_angle == -1 or rightmost_angle == -1:
            return False

        nearest_point = self.__angle_to_xy(nearest_angle)
        rightmost_point = self.__angle_to_xy(rightmost_angle)

        wall_angle = self.__angle_from_points(nearest_point, rightmost_point)
        if wall_angle > 100:
            return False

        nearest_dist = self.__lidar.scan_data[nearest_angle]
        rightmost_dist = self.__lidar.scan_data[rightmost_angle]

        if rightmost_dist <= nearest_dist:
            return False

        offset = -self.__speed_controller.get_braking_distance()
        offset += config["parking"]["start_steering_offset"]

        distance = math.sqrt(rightmost_dist**2 - nearest_dist**2)
        threshold = (config["kart"]["dimensions"]["length"] - config["kart"]["lidar_offset"] + offset) * 1000

        return distance > threshold

    def __start_steering(self) -> None:
        """Start steering into the parking spot."""
        # Stop the go-kart before proceeding
        self.__speed_controller.toggle()
        time.sleep(1)

        # Reverse into the parking spot
        self.__can_controller.set_steering(1.25)
        time.sleep(1)

        self.__speed_controller.toggle()
        self.__speed_controller.gear = Gear.REVERSE

    def __start_steering_back(self) -> None:
        """Start steering back into the parking spot."""
        # Stop the go-kart before proceeding
        self.__speed_controller.toggle()
        time.sleep(1)

        # Turn the wheels all the way to the left
        self.__can_controller.set_steering(-1.25)
        time.sleep(1)

        self.__speed_controller.toggle()
        self.__speed_controller.gear = Gear.REVERSE

    def __straighten_wheels(self) -> None:
        """Straighten the wheels."""
        self.__can_controller.set_steering(0)

    def __wait_until_opening(self, consecutive: int = 3) -> None:
        """Wait until the opening is reached.

        :param consecutive: The amount of consecutive scans that need to detect an opening.
        """
        found_opening = 0
        while found_opening < consecutive:
            if self.__lidar.free_range(265, 290, 3000):
                found_opening += 1
            else:
                found_opening = 0

            time.sleep(0.1)

    def __wait_until_wall(self, consecutive: int = 3, estimate_length: bool = False) -> None:
        """Wait until the lidar sensor detects a wall.

        :param consecutive: The amount of consecutive scans that need to detect a wall.
        :param estimate_length: Whether to estimate the length of the parking space.
        """
        found_wall = 0
        while found_wall < consecutive:
            if estimate_length:
                self.__estimate_length()

            if self.__lidar.free_range(265, 275, 3000):
                found_wall = 0
            else:
                found_wall += 1

            time.sleep(0.1)

    @staticmethod
    def __angle_from_points(a: tuple[float, float], b: tuple[float, float]) -> float:
        """Calculate the angle between two points.

        :param a: The first point.
        :param b: The second point.
        :return: The angle between the two points.
        """
        dx = b[0] - a[0]
        dy = b[1] - a[1]

        angle = math.degrees(math.atan2(dy, dx))
        return (180 - angle) % 360

    @staticmethod
    def __wait_until(condition: Callable[[], bool]) -> None:
        """Wait until a certain condition is met.

        :param condition: The condition to check.
        """
        while not condition():
            time.sleep(0.01)
