import logging
import time

from threading import Thread
from ultralytics.engine.results import Boxes

from src.config import config
from src.constants import Label
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.object_recognition.object_controller import ObjectController
from src.utils.lidar import BaseLidar


class OvertakeHandler(BaseObjectHandler):
    """A handler for overtaking vehicles.

    Attributes
    ----------
        lidar (Lidar): The lidar sensor.

    """

    lidar: BaseLidar
    __frames_seen: dict[int, int]
    __frames_lost: dict[int, int]
    __thread: Thread

    def __init__(self, controller: ObjectController, lidar: BaseLidar) -> None:
        """Initializes the overtaking handler.

        :param controller: The object controller.
        :param lidar: The lidar sensor.
        """
        super().__init__(controller, [Label.CAR])
        self.lidar = lidar
        self.__frames_seen = {}
        self.__frames_lost = {}
        self.__thread = Thread(target=self.__return_lane, daemon=True)
        self.__thread.start()

    def handle(self, predictions: Boxes) -> None:
        """Handles the detected overtaking vehicles.

        :param predictions: The detected overtaking vehicles (.data: x1, y1, x2, y2, track_id, conf, cls).
        """
        full_lanes = set()
        for x1, _, x2, y2 in predictions.xyxy:
            cx = (x1 + x2) // 2
            distance = self.controller.calibration.get_distance_to_y(cx, y2, predictions.orig_shape[::-1])
            reaction_distance = self.controller.get_reaction_distance()

            total_distance = distance - reaction_distance
            if total_distance > config["overtake"]["min_distance"]:
                continue

            lane = self.controller.get_object_lane(cx, y2, predictions.orig_shape[::-1])
            if lane is None:
                continue

            full_lanes.add(lane)

        current_lane = self.controller.get_current_lane()
        if current_lane in full_lanes:
            logging.info("Vehicle detected in the current lane. Switching to the next lane.")
            self.controller.set_lane(current_lane + 1)

            if config["overtake"]["force_move"]["enabled"]:
                # Force the go-kart to switch to the next lane
                self.controller.lane_assist.toggle()
                self.controller.set_steering(config["overtake"]["force_move"]["angle"])

                # Wait for the specified duration
                time.sleep(config["overtake"]["force_move"]["duration"])

                # Reset the steering angle to 0.0
                self.controller.set_steering(0.0)
                self.controller.lane_assist.toggle()

    def __return_lane(self) -> None:
        """Checks if the side is free and returns to the previous lane."""
        while True:
            current_lane = self.controller.get_current_lane()
            if current_lane == 0:
                time.sleep(0.1)
                continue

            if current_lane not in self.__frames_seen:
                self.__frames_lost[current_lane] = 0
                self.__frames_seen[current_lane] = 0

            is_side_free = self.lidar.free_range(
                config["overtake"]["min_angle"],
                config["overtake"]["max_angle"],
                config["overtake"]["range_threshold"]
            )

            # Wait until we have seen the car for the first time
            if not is_side_free:
                self.__frames_seen[current_lane] += 1

            # If we have previously seen the car, we can start checking if we have passed it
            if self.__frames_seen[current_lane] >= config["overtake"]["consecutive_frames"]:
                if is_side_free:
                    self.__frames_lost[current_lane] += 1
                else:
                    self.__frames_lost[current_lane] -= 1

            # If the side is free for a certain amount of frames, return to the previous lane
            if self.__frames_lost[current_lane] >= config["overtake"]["consecutive_frames"]:
                logging.info("The right side is free. Returning to the previous lane.")

                self.controller.set_lane(current_lane - 1)
                del self.__frames_lost[current_lane]
                del self.__frames_seen[current_lane]

                if config["overtake"]["force_return"]["enabled"]:
                    # Force the go-kart to return to the previous lane
                    self.controller.lane_assist.toggle()
                    self.controller.set_steering(config["overtake"]["force_return"]["angle"])

                    # Wait for the specified duration
                    time.sleep(config["overtake"]["force_return"]["duration"])

                    # Reset the steering angle to 0.0
                    self.controller.set_steering(0.0)
                    self.controller.lane_assist.toggle()

            time.sleep(0.1)
