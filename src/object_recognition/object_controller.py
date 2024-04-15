from driving.speed_controller import SpeedControllerState
from driving.speed_controller.speed_controller_interface import ISpeedController
from object_recognition.handlers.base_handler import BaseObjectHandler
from ultralytics.engine.results import Boxes


class ObjectController:
    """A controller for the objects detected by the object detector.

    Attributes
    ----------
        handlers (list[BaseObjectHandler]): The object handlers.
        speed_controller (SpeedController): The speed controller.
        stopped_by (BaseObjectHandler): The handler that stopped the go-kart.

    """

    handlers: list[BaseObjectHandler]
    speed_controller: ISpeedController
    stopped_by: BaseObjectHandler = None

    def __init__(self, speed_controller: ISpeedController) -> None:
        """Initializes the object controller.

        :param speed_controller: The speed controller.
        """
        self.handlers = []
        self.speed_controller = speed_controller

    def add_handler(self, handler: BaseObjectHandler) -> None:
        """Adds a handler to the object controller.

        :param handler: The handler to add.
        """
        self.handlers.append(handler)

    def get_braking_distance(self) -> float:
        """Calculates the braking distance of the go-kart.

        :return: The braking distance.
        """
        return self.speed_controller.get_braking_distance()

    def handle(self, predictions: Boxes) -> None:
        """Handles the predictions.

        :param predictions: The predictions to handle.
        """
        for handler in self.handlers:
            filtered_predictions = handler.filter_predictions(predictions)
            if len(filtered_predictions.data) == 0:
                continue

            handler.handle(filtered_predictions)

    def has_stopped(self) -> bool:
        """Checks if the go-kart has stopped.

        :return: Whether the go-kart has stopped.
        """
        return self.speed_controller.state == SpeedControllerState.STOPPED

    def set_max_speed(self, speed: int) -> None:
        """Sets the maximum speed of the go-kart.

        :param speed: The maximum speed.
        """
        self.speed_controller.max_speed = speed

    def set_state(self, state: SpeedControllerState) -> None:
        """Sets the state of the speed controller.

        :param state: The new state.
        """
        self.speed_controller.state = state
