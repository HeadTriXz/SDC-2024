import numpy as np

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING
from ultralytics.engine.results import Boxes

if TYPE_CHECKING:
    from src.object_recognition.object_controller import ObjectController


class BaseObjectHandler(ABC):
    """Base class for all object handlers.

    Attributes
    ----------
        allowed_classes (list[int]): The allowed classes for the handler.

    """

    allowed_classes: list[int]
    controller: "ObjectController"

    def __init__(self, controller: "ObjectController", allowed_classes: list[int]) -> None:
        """Initializes the handler.

        :param controller: The object controller.
        :param allowed_classes: The allowed classes for the handler.
        """
        self.allowed_classes = allowed_classes
        self.controller = controller

    def filter_predictions(self, predictions: Boxes) -> Boxes:
        """Filters the predictions.

        :param predictions: The predictions to filter.
        :return: The filtered predictions.
        """
        data = predictions.data.numpy()
        data = data[np.in1d(data[:, -1], self.allowed_classes)]

        return Boxes(data, predictions.orig_shape)

    def is_stopped_by_other(self) -> bool:
        """Checks if another handler has stopped the go-kart.

        :return: Whether the handler has stopped the go-kart.
        """
        return self.controller.has_stopped() and self.controller.stopped_by != self

    @staticmethod
    def get_closest_prediction(predictions: Boxes) -> np.ndarray | None:
        """Gets the closest prediction to the go-kart. We assume the largest bounding box is the closest.

        :param predictions: The predictions to search.
        :return: The closest prediction.
        """
        largest_area = 0
        closest_prediction = None

        for box in predictions.data:
            area = box[2] * box[3]
            if area > largest_area:
                largest_area = area
                closest_prediction = box

        return closest_prediction

    @abstractmethod
    def handle(self, predictions: Boxes) -> None:
        """Handles the predictions.

        :param predictions: The predictions to handle.
        """
        pass
