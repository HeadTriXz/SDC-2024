import numpy as np

from config import config
from constants import Label
from driving.speed_controller import SpeedControllerState
from object_recognition.handlers.base_handler import BaseObjectHandler
from object_recognition.object_controller import ObjectController
from ultralytics.engine.results import Boxes


class PedestrianHandler(BaseObjectHandler):
    """A handler for crossing pedestrians.

    Attributes
    ----------
        track_history (dict[int, list[int]]): The history of the pedestrians.

    """

    track_history: dict[int, np.ndarray]

    def __init__(self, controller: ObjectController) -> None:
        """Initializes the pedestrian handler.

        :param controller: The object controller.
        """
        super().__init__(controller, [Label.PERSON, Label.CROSSWALK])
        self.track_history = {}

    def handle(self, predictions: Boxes) -> None:
        """Handles the detected pedestrians.

        :param predictions: The detected pedestrians (.data: x1, y1, x2, y2, track_id, conf, cls).
        """
        if self.is_stopped_by_other():
            return

        stopped = self.controller.has_stopped()
        should_stop = self.__should_stop(predictions)

        if stopped and not should_stop:
            self.controller.stopped_by = None
            self.controller.set_state(SpeedControllerState.DRIVING)
        elif not stopped and should_stop:
            self.controller.stopped_by = self
            self.controller.set_state(SpeedControllerState.STOPPED)

    def __get_direction(self, history: np.ndarray) -> int:
        """Calculates the direction of the pedestrian.

        :param history: The pedestrian's previous positions.
        :return: The direction of the pedestrian. -1 if moving left, 1 if moving right.
        """
        weights = np.linspace(1, 0, len(history))
        average = np.average(history[:, 0], weights=weights)

        return np.sign(history[-1][0] - average)

    def __get_most_recent_side(self, history: np.ndarray) -> int:
        """Calculates the most recent side the pedestrian was on.

        :param history: The pedestrian's previous positions.
        :return: The most recent side the pedestrian was on. -1 if left, 1 if right, 0 if in the middle.
        """
        margin = config.crosswalk.safe_zone_margin
        known_sides = []

        for position in history[::-1]:
            if len(known_sides) == 2:
                return known_sides[1]

            if position[0] < margin:
                known_sides.append(-1)

            if position[0] > 1 - margin:
                known_sides.append(1)

        return 0

    def __get_relative_position(self, crosswalk: np.ndarray, pedestrian: np.ndarray) -> np.ndarray:
        """Calculates the relative position of the pedestrian to the crosswalk.

        :param crosswalk: The bounding box of the crosswalk.
        :param pedestrian: The bounding box of the pedestrian.
        :return: The relative position of the pedestrian (x1, y1, x2, y2).
        """
        width = crosswalk[2] - crosswalk[0]
        height = crosswalk[3] - crosswalk[1]

        return np.array([
            (pedestrian[0] - crosswalk[0]) / width,
            (pedestrian[1] - crosswalk[1]) / height,
            (pedestrian[2] - crosswalk[0]) / width,
            (pedestrian[3] - crosswalk[1]) / height
        ])

    def __overlaps(self, pedestrian: np.ndarray) -> bool:
        """Checks if the bounding boxes overlap.

        :param pedestrian: The relative coordinates of the pedestrian.
        :return: Whether the bounding boxes overlap.
        """
        min_margin = -config.crosswalk.overlap_margin
        max_margin = 1 - min_margin

        return (min_margin <= pedestrian[3] <= max_margin and
                (min_margin <= pedestrian[0] <= max_margin or
                 min_margin <= pedestrian[2] <= max_margin))

    def __reached_safe_zone(self, history: np.ndarray) -> bool:
        """Checks if the pedestrian has reached the safe zone.

        :param history: The pedestrian's previous positions.
        :return: Whether the pedestrian has reached the safe zone.
        """
        if len(history) < 2:
            return False

        side = self.__get_most_recent_side(history)
        direction = self.__get_direction(history)
        if direction == side:
            return False

        margin = config.crosswalk.safe_zone_margin
        if direction == -1:
            return history[-1][0] < margin

        return history[-1][0] > 1 - margin

    def __should_brake(self, crosswalk: np.ndarray) -> bool:
        """Checks if the crosswalk is close enough to stop.

        :param crosswalk: The bounding box of the crosswalk.
        :return: Whether the crosswalk is close enough to stop.
        """
        return crosswalk[3] > 330  # TODO: Dynamically calculate the distance

    def __should_stop(self, predictions: Boxes) -> bool:
        """Checks if the go-kart should stop.

        :param predictions: The detected pedestrians.
        :return: Whether the go-kart should stop.
        """
        if not predictions.is_track:
            return False

        crosswalks = predictions.data[predictions.cls == Label.CROSSWALK]
        pedestrians = predictions.data[predictions.cls == Label.PERSON]

        if len(crosswalks) == 0 or len(pedestrians) == 0:
            return False

        for crosswalk in crosswalks:
            if not self.__should_brake(crosswalk):
                continue

            for pedestrian in pedestrians:
                relative_position = self.__get_relative_position(crosswalk, pedestrian)
                if not self.__overlaps(relative_position):
                    continue

                centroid = self.__xyxy_to_centroid(relative_position[:4])

                p_id = int(pedestrian[4])
                if p_id not in self.track_history:
                    self.track_history[p_id] = np.array([centroid])
                else:
                    self.track_history[p_id] = np.append(self.track_history[p_id], [centroid], axis=0)
                    if self.__reached_safe_zone(self.track_history[p_id]):
                        continue

                return True

        return False

    def __xyxy_to_centroid(self, xyxy: np.ndarray) -> np.ndarray:
        """Converts the bounding box from (x1, y1, x2, y2) to (x, y).

        :param xyxy: The bounding box.
        :return: The centroid of the bounding box.
        """
        return np.array([(xyxy[0] + xyxy[2]) / 2, xyxy[3]])
