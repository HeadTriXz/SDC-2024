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
        allowed_classes = list(config.object_detection.class_to_speed.keys())
        super().__init__(controller, allowed_classes)

    def handle(self, predictions: Boxes) -> None:
        """Updates the speed limit based on the detected objects.

        :param predictions: The detected speed limit signs.
        """
        closest_class = self.get_closest_prediction(predictions)
        if closest_class is None:
            return

        speed = config.object_detection.class_to_speed[closest_class]
        self.controller.set_max_speed(speed)
