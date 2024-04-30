import numpy as np

from driving.speed_controller import SpeedControllerState
from driving.speed_controller.speed_controller_interface import ISpeedController
from lane_assist.lane_assist import LaneAssist
from object_recognition.handlers.base_handler import BaseObjectHandler
from typing import Optional
from ultralytics.engine.results import Boxes
from utils.calibration_data import CalibrationData


class ObjectController:
    """A controller for the objects detected by the object detector.

    Attributes
    ----------
        calibration (CalibrationData): The calibration data.
        handlers (list[BaseObjectHandler]): The object handlers.
        lane_assist (LaneAssist): The lane assist.
        speed_controller (SpeedController): The speed controller.
        stopped_by (BaseObjectHandler): The handler that stopped the go-kart.

    """

    calibration: CalibrationData
    handlers: list[BaseObjectHandler]
    lane_assist: LaneAssist
    speed_controller: ISpeedController
    stopped_by: BaseObjectHandler = None

    def __init__(
            self,
            calibration: CalibrationData,
            lane_assist: LaneAssist,
            speed_controller: ISpeedController
    ) -> None:
        """Initializes the object controller.

        :param calibration: The calibration data.
        :param lane_assist: The lane assist.
        :param speed_controller: The speed controller.
        """
        self.calibration = calibration
        self.handlers = []
        self.lane_assist = lane_assist
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

    def get_current_lane(self) -> int:
        """Gets the current lane of the go-kart.

        :return: The current lane.
        """
        return self.lane_assist.requested_lane

    def get_object_lane(self, x: int, y: int, shape: tuple[int, int]) -> Optional[int]:
        """Gets the lane the object is in.

        :param x: The x-coordinate of the object.
        :param y: The y-coordinate of the object.
        :param shape: The shape of the image (width, height).
        :return: The lane the object is in.
        """
        point = np.array(self.calibration.transform_point(x, y, shape))

        def is_left(p0: np.ndarray, p1: np.ndarray, p: np.ndarray) -> bool:
            return np.sign(np.cross(p1 - p0, p - p0)) > 0

        for i in range(len(self.lane_assist.lines) - 1):
            line1 = self.lane_assist.lines[i]
            line2 = self.lane_assist.lines[i + 1]

            line_left1 = np.all([is_left(line1.points[i], line1.points[i + 1], point)
                                 for i in range(len(line1.points) - 1)])
            line_left2 = np.all([is_left(line2.points[i], line2.points[i + 1], point)
                                 for i in range(len(line2.points) - 1)])

            any_close = [np.allclose(point, p) for p in np.vstack((line1.points, line2.points))]

            if (line_left1 != line_left2) and not any(any_close):
                return i

        return None

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

    def set_lane(self, lane: int) -> None:
        """Sets the lane to drive in.

        :param lane: The lane to drive in.
        """
        self.lane_assist.requested_lane = lane

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
