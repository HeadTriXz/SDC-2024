import numpy as np
import time

from ultralytics.engine.results import Boxes

from src.config import config
from src.constants import Label
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.object_recognition.object_controller import ObjectController
from src.utils.lidar import BaseLidar
from src.utils.parking import ParkingManoeuvre


class ParkingHandler(BaseObjectHandler):
    """A handler for parking spaces."""

    __lidar: BaseLidar

    def __init__(self, controller: ObjectController, lidar: BaseLidar) -> None:
        """Initializes the parking handler.

        :param controller: The object controller.
        :param lidar: The lidar sensor.
        """
        super().__init__(controller, [Label.PARKING_SPACE])
        self.__lidar = lidar

    def handle(self, predictions: Boxes) -> None:
        """Check if there is a parking space available. If so, set the state to parking.

        :param predictions: The detected parking spaces.
        """
        if self.is_stopped_by_other():
            return

        if not self.__any_within_distance(predictions):
            return

        self.controller.stop()

        manoeuvre = ParkingManoeuvre(self.__lidar, self.controller.lane_assist)
        manoeuvre.park()

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

    def __is_within_distance(self, bbox: np.ndarray, shape: tuple[int, int]) -> bool:
        """Check if the parking space is within the distance threshold.

        :param bbox: The bounding box of the parking space.
        :param shape: The shape of the image (height, width).
        :return: Whether the parking space is within the distance threshold.
        """
        x, y = (bbox[0] + bbox[2]) // 2, bbox[3]
        distance = self.controller.calibration.get_distance_to_y(x, y, shape[::-1])

        return distance < config["parking"]["min_distance"]

    @staticmethod
    def __is_on_my_right(bbox: np.ndarray, shape: tuple[int, int]) -> bool:
        """Check if the parking space is on the right side of the go-kart.

        :param bbox: The bounding box of the parking space.
        :param shape: The shape of the image (height, width).
        :return: Whether the parking space is on the right side of the go-kart.
        """
        return bbox[0] > shape[1] // 2
