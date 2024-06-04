import logging
import numpy as np

from ultralytics.engine.results import Boxes

from src.config import config
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.object_recognition.object_controller import ObjectController


class SpeedLimitHandler(BaseObjectHandler):
    """A handler for the speed limit signs."""

    def __init__(self, controller: ObjectController) -> None:
        """Initializes the speed limit handler.

        :param controller: The object controller.
        """
        allowed_classes = list(config["speed_limit"]["class_to_speed"].keys())
        super().__init__(controller, allowed_classes)

    def handle(self, predictions: Boxes) -> None:
        """Updates the speed limit based on the detected objects.

        :param predictions: The detected speed limit signs.
        """
        closest = self.get_closest_prediction(predictions)
        if closest is None:
            return

        x, y = self.__get_sign_coords(closest)
        distance = self.controller.calibration.get_distance_to_y(x, y, predictions.orig_shape[::-1])

        stopping_distance = self.controller.get_stopping_distance()
        total_distance = distance - stopping_distance

        if total_distance > config["speed_limit"]["min_distance"]:
            return

        speed = config["speed_limit"]["class_to_speed"][int(closest[-1])]
        logging.info("Speed limit sign detected. Setting the speed limit to %d km/h.", speed)

        self.controller.set_max_speed(speed)

    @staticmethod
    def __get_sign_coords(bbox: np.ndarray) -> tuple[int, int]:
        """Get the coordinates of where the speed limit sign is on the ground.

        :param bbox: The bounding box of the speed limit sign.
        :return: The coordinates of the speed limit sign.
        """
        height = (bbox[3] - bbox[1]) * config["speed_limit"]["height_ratio"]

        x = bbox[0] + bbox[2] // 2
        y = bbox[1] + height

        return x, y
