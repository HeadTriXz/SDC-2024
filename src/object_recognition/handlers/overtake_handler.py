from config import config
from constants import Label
from object_recognition.handlers.base_handler import BaseObjectHandler
from object_recognition.object_controller import ObjectController
from threading import Thread
from ultralytics.engine.results import Boxes
from utils.lidar import Lidar


class OvertakeHandler(BaseObjectHandler):
    """A handler for overtaking vehicles.

    Attributes
    ----------
        lidar (Lidar): The lidar sensor.

    """

    lidar: Lidar
    __frames_seen: dict[int, int]
    __frames_lost: dict[int, int]
    __thread: Thread

    def __init__(self, controller: ObjectController) -> None:
        """Initializes the overtaking handler.

        :param controller: The object controller.
        """
        super().__init__(controller, [Label.CAR])
        self.lidar = Lidar()
        self.__frames_seen = {}
        self.__frames_lost = {}
        self.__thread = Thread(target=self.__return_lane, daemon=True)
        self.__thread.start()

    def handle(self, predictions: Boxes) -> None:
        """Handles the detected overtaking vehicles.

        :param predictions: The detected overtaking vehicles (.data: x1, y1, x2, y2, track_id, conf, cls).
        """
        full_lanes = set()
        for x1, x2, _, y2 in predictions.xyxy:
            cx = (x1 + x2) // 2
            distance = self.controller.calibration.get_distance_to_y(cx, y2, predictions.orig_shape[::-1])
            if distance > config.overtake.min_distance:
                continue

            lane = self.controller.get_object_lane(cx, y2, predictions.orig_shape[::-1])
            if lane is None:
                continue

            full_lanes.add(lane)

        current_lane = self.controller.get_current_lane()
        if current_lane in full_lanes:
            self.controller.set_lane(current_lane + 1)

    def __return_lane(self) -> None:
        """Checks if the side is free and returns to the previous lane."""
        while True:
            current_lane = self.controller.get_current_lane()
            if current_lane == 0:
                continue

            if current_lane not in self.__frames_seen:
                self.__frames_lost[current_lane] = 0
                self.__frames_seen[current_lane] = 0

            is_side_free = self.lidar.free_range(
                config.overtake.min_angle,
                config.overtake.max_angle,
                config.overtake.range_threshold
            )

            if not is_side_free:
                self.__frames_seen[current_lane] += 1

            if self.__frames_seen[current_lane] >= config.overtake.consecutive_frames:
                if is_side_free:
                    self.__frames_lost[current_lane] += 1
                else:
                    self.__frames_lost[current_lane] -= 1

            if self.__frames_lost[current_lane] >= config.overtake.consecutive_frames:
                self.controller.set_lane(current_lane - 1)
                del self.__frames_lost[current_lane]
                del self.__frames_seen[current_lane]
