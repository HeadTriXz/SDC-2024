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

        bottom_y = int(closest[3] * config.speed_limit.height_ratio)
        centroid_x = (closest[0] + closest[2]) // 2

        distance = self.controller.calibration.get_distance_to_point(centroid_x, bottom_y, False)
        braking_distance = self.controller.get_braking_distance()
        total_distance = distance - braking_distance

        if total_distance > config.speed_limit.min_distance:
            return

        speed = config.speed_limit.class_to_speed[int(closest[-1])]
        self.controller.set_max_speed(speed)
