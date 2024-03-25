from constants import Label
from driving.speed_controller import SpeedControllerState
from object_recognition.handlers.base_handler import BaseObjectHandler
from object_recognition.object_controller import ObjectController
from ultralytics.engine.results import Boxes


class TrafficLightHandler(BaseObjectHandler):
    """A handler for the traffic lights."""

    def __init__(self, controller: ObjectController) -> None:
        """Initializes the traffic light handler.

        :param controller: The object controller.
        """
        super().__init__(controller, [Label.TRAFFIC_LIGHT_RED, Label.TRAFFIC_LIGHT_GREEN])

    def handle(self, predictions: Boxes) -> None:
        """Sets the state of the speed controller based on the detected traffic light.

        :param predictions: The detected traffic light.
        """
        closest_class = self.get_closest_prediction(predictions)
        if closest_class is None:
            return

        match closest_class:
            case Label.TRAFFIC_LIGHT_RED:
                self.controller.set_state(SpeedControllerState.WAITING_TO_STOP)
            case Label.TRAFFIC_LIGHT_GREEN:
                self.controller.set_state(SpeedControllerState.DRIVING)
            case _:
                pass
