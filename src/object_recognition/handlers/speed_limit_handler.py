import numpy as np

from config import config
from object_recognition.handlers.base_handler import BaseObjectHandler
from object_recognition.object_controller import ObjectController
from ultralytics.engine.results import Boxes


class SpeedLimitHandler(BaseObjectHandler):
    """A handler for the speed limit signs."""

    def __init__(self, controller: ObjectController) -> None:
        """Initializes the speed limit handler.

        :param controller: The object controller.
        """
        allowed_classes = list(config.speed_limit.class_to_speed.keys())
        super().__init__(controller, allowed_classes)

    def handle(self, predictions: Boxes) -> None:
        """Updates the speed limit based on the detected objects.

        :param predictions: The detected speed limit signs.
        """
        closest = self.get_closest_prediction(predictions)
        if closest is None:
            return

        new_y = self._get_sign_coords(closest, predictions.orig_shape[::-1])[1]

        bottom = self.controller.calibration.output_shape[1]
        distance = self.controller.calibration.get_distance(int(bottom - new_y))

        braking_distance = self.controller.get_braking_distance()
        total_distance = distance - braking_distance

        if total_distance > config.speed_limit.min_distance:
            return

        speed = config.speed_limit.class_to_speed[int(closest[-1])]
        self.controller.set_max_speed(speed)

    def _get_sign_coords(self, bbox: np.ndarray, shape: tuple[int, int]) -> tuple[int, int]:
        """Get the coordinates of where the speed limit sign is on the ground.

        :param bbox: The bounding box of the speed limit sign.
        :param shape: The shape of the image (width, height).
        :return: The coordinates of the speed limit sign.
        """
        height = (bbox[3] - bbox[1]) * config.speed_limit.height_ratio

        max_y = bbox[1] + height
        ctr_x = bbox[0] + bbox[2] // 2

        return self.controller.calibration.transform_point(ctr_x, max_y, shape)
