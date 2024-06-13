import logging
import time

from ultralytics.engine.results import Boxes

from src.config import config
from src.constants import Label
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.object_recognition.object_controller import ObjectController
from src.utils.lidar import Lidar


class OvertakeHandler(BaseObjectHandler):
    """A handler for overtaking vehicles.

    Attributes
    ----------
        lidar (Lidar): The lidar sensor.

    """

    lidar: Lidar
    __known_vehicles: set[int]

    def __init__(self, controller: ObjectController, lidar: Lidar) -> None:
        """Initializes the overtaking handler.

        :param controller: The object controller.
        :param lidar: The lidar sensor.
        """
        super().__init__(controller, [Label.CAR])

        self.lidar = lidar
        self.__known_vehicles = set()

    def handle(self, predictions: Boxes) -> None:
        """Handles the detected overtaking vehicles.

        :param predictions: The detected overtaking vehicles (.data: x1, y1, x2, y2, track_id, conf, cls).
        """
        if not predictions.is_track:
            return

        full_lanes = self.__get_full_lanes(predictions)
        current_lane = self.controller.get_current_lane()
        if current_lane not in full_lanes:
            return

        logging.info("Vehicle detected in the current lane. Switching to the next lane.")
        self.__switch_lane(1)

        for track_id in full_lanes[current_lane]:
            self.__known_vehicles.add(track_id)

        self.__wait_until_seen(config["overtake"]["consecutive_scans"])
        self.__wait_until_safe(config["overtake"]["consecutive_scans"])

        logging.info("The right lane is free. Returning to the previous lane.")
        self.__switch_lane(0)

    def __get_full_lanes(self, predictions: Boxes) -> dict[int, list[int]]:
        """Gets the lanes with detected vehicles.

        :param predictions: The detected vehicles.
        :return: The lanes with detected vehicles.
        """
        full_lanes: dict[int, list[int]] = {}
        for x1, _, x2, y2, track_id, _, _ in predictions.data:
            if track_id in self.__known_vehicles:
                continue

            cx = (x1 + x2) // 2

            distance = self.controller.calibration.get_distance_to_y(cx, y2, predictions.orig_shape[::-1])
            reaction_distance = self.controller.get_reaction_distance()

            total_distance = distance - reaction_distance
            if total_distance > config["overtake"]["min_distance"]:
                continue

            lane = self.controller.get_object_lane(cx, y2, predictions.orig_shape[::-1])
            if lane is not None:
                if lane not in full_lanes:
                    full_lanes[lane] = []

                full_lanes[lane].append(track_id)

        return full_lanes

    def __is_safe_to_return(self) -> bool:
        """Checks if it is safe to return to the original lane.

        :return: Whether it is safe to return to the original lane.
        """
        return self.lidar.free_range(
            config["overtake"]["min_angle"],
            config["overtake"]["max_angle"],
            config["overtake"]["range_threshold"] * 1000,
            config["overtake"]["max_points"]
        )

    def __switch_lane(self, lane: int) -> None:
        """Switch to the specified lane.

        :param lane: The lane to switch to.
        """
        is_left = lane > self.controller.get_current_lane()
        config_key = "force_move" if is_left else "force_return"

        self.controller.set_lane(lane)

        if config["overtake"][config_key]["enabled"]:
            self.controller.lane_assist.toggle()
            self.controller.set_steering(config["overtake"][config_key]["angle"])

            time.sleep(config["overtake"][config_key]["duration"])
            self.controller.set_steering(0.0)

            time.sleep(config["overtake"][config_key]["straight_duration"])
            self.controller.lane_assist.toggle()

    def __wait_until_safe(self, consecutive: int = 3) -> None:
        """Wait until it is safe to return to the original lane.

        :param consecutive: The number of consecutive scans the lane should be free.
        """
        count = 0
        while count < consecutive:
            if self.__is_safe_to_return():
                count += 1
            else:
                count = 0

            time.sleep(0.1)

    def __wait_until_seen(self, consecutive: int = 3) -> None:
        """Wait until the vehicle is seen.

        :param consecutive: The number of consecutive scans the vehicle should be detected.
        """
        count = 0
        while count < consecutive:
            if self.__is_safe_to_return():
                count = 0
            else:
                count += 1

            time.sleep(0.1)
