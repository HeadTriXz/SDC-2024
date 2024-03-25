import numpy as np

from config import crosswalk_overlap_margin, crosswalk_safe_zone_margin
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
        crosswalks = predictions.data[predictions.cls == Label.CROSSWALK]
        pedestrians = predictions.data[predictions.cls == Label.PERSON]

        print(f"Found {len(pedestrians)} pedestrians")

        if len(crosswalks) == 0 or len(pedestrians) == 0:
            return

        should_wait = False
        for crosswalk in crosswalks:
            if should_wait:
                break

            if not self.__should_brake(crosswalk):
                continue

            for pedestrian in pedestrians:
                if not self.__overlaps(crosswalk, pedestrian):
                    continue

                centroid = self.__xyxy_to_centroid(pedestrian[:4])

                p_id = pedestrian[4]
                if p_id not in self.track_history:
                    self.track_history[p_id] = np.array([centroid])
                else:
                    self.track_history[p_id] = np.append(self.track_history[p_id], [centroid], axis=0)
                    if self.__reached_safe_zone(crosswalk, self.track_history[p_id]):
                        continue

                should_wait = True
                break

        if should_wait:
            self.controller.set_state(SpeedControllerState.STOPPED)
        else:
            self.controller.set_state(SpeedControllerState.DRIVING)

    def __get_direction(self, history: np.ndarray) -> int:
        """Calculates the direction of the pedestrian.

        :param history: The pedestrian's previous positions.
        :return: The direction of the pedestrian. -1 if moving left, 1 if moving right.
        """
        weights = np.linspace(1, 0, len(history))
        average = np.average(history[:, 0], weights=weights)

        return np.sign(history[-1][0] - average)

    def __get_initial_side(self, crosswalk: np.ndarray, history: np.ndarray) -> int:
        """Calculates the initial side the pedestrian started from.

        :param crosswalk: The bounding box of the crosswalk.
        :param history: The pedestrian's previous positions.
        :return: The initial side the pedestrian started from. -1 if left, 1 if right.
        """
        margin_x = (crosswalk[2] - crosswalk[0]) * crosswalk_safe_zone_margin

        if history[0][0] < crosswalk[0] - margin_x:
            return -1
        elif history[0][0] > crosswalk[2] + margin_x:
            return 1

        return 0

    def __overlaps(self, crosswalk: np.ndarray, pedestrian: np.ndarray) -> bool:
        """Checks if the bounding boxes overlap.

        :param crosswalk: The bounding box of the crosswalk.
        :param pedestrian: The bounding box of the pedestrian.
        :return: True if the bounding boxes overlap, False otherwise.
        """
        margin = (crosswalk[3] - crosswalk[1]) * crosswalk_overlap_margin
        min_y_crosswalk = crosswalk[1] - margin
        max_y_crosswalk = crosswalk[3] + margin

        return (min_y_crosswalk <= pedestrian[3] <= max_y_crosswalk and
                (crosswalk[0] <= pedestrian[0] <= crosswalk[2] or
                 crosswalk[0] <= pedestrian[2] <= crosswalk[2]))

    def __reached_safe_zone(self, crosswalk: np.ndarray, history: np.ndarray) -> bool:
        """Checks if the pedestrian has reached the safe zone.

        :param crosswalk: The bounding box of the crosswalk.
        :param history: The pedestrian's previous positions.
        :return: Whether the pedestrian has reached the safe zone.
        """
        if len(history) < 2:
            return False

        side = self.__get_initial_side(crosswalk, history)
        direction = self.__get_direction(history)
        if direction == side:
            return False

        margin_x = (crosswalk[2] - crosswalk[0]) * crosswalk_safe_zone_margin
        if direction == -1:
            return history[-1][0] < crosswalk[0] + margin_x
        else:
            return history[-1][0] > crosswalk[2] - margin_x

    def __should_brake(self, crosswalk: np.ndarray) -> bool:
        """Checks if the crosswalk is close enough to stop.

        :param crosswalk: The bounding box of the crosswalk.
        :return: Whether the crosswalk is close enough to stop.
        """
        return True

    def __xyxy_to_centroid(self, xyxy: np.ndarray) -> np.ndarray:
        """Converts the bounding box from (x1, y1, x2, y2) to (x, y).

        :param xyxy: The bounding box.
        :return: The centroid of the bounding box.
        """
        return np.array([(xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2])
