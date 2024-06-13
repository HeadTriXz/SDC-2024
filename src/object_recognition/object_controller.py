import numpy as np

from ultralytics.engine.results import Boxes

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.speed_controller import ISpeedController, SpeedControllerState
from src.lane_assist.lane_assist import LaneAssist
from src.object_recognition.handlers.base_handler import BaseObjectHandler
from src.utils.other import is_point_between


class ObjectController:
    """A controller for the objects detected by the object detector.

    Attributes
    ----------
        calibration (CalibrationData): The calibration data.
        disabled (bool): Whether the object controller is disabled.
        handlers (list[BaseObjectHandler]): The object handlers.
        lane_assist (LaneAssist): The lane assist.
        speed_controller (SpeedController): The speed controller.
        stopped_by (BaseObjectHandler): The handler that stopped the go-kart.

    """

    calibration: CalibrationData
    disabled: bool = False
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

    def stop(self) -> None:
        """Stops checking for new objects."""
        self.disabled = True

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

    def get_object_lane(self, x: int, y: int, shape: tuple[int, int]) -> int | None:
        """Gets the lane the object is in.

        :param x: The x-coordinate of the object.
        :param y: The y-coordinate of the object.
        :param shape: The shape of the image (width, height).
        :return: The lane the object is in.
        """
        if len(self.lane_assist.lines) < 2:
            return None

        point = np.array(self.calibration.transform_point(x, y, shape))
        for i in range(len(self.lane_assist.lines) - 1):
            line_left = self.lane_assist.lines[i].points
            line_right = self.lane_assist.lines[i + 1].points

            closest_left = np.argmin(np.linalg.norm(line_left - point, axis=1))
            closest_right = np.argmin(np.linalg.norm(line_right - point, axis=1))

            if is_point_between(line_left[closest_left], line_right[closest_right], point):
                return len(self.lane_assist.lines) - i - 2

        return None

    def get_reaction_distance(self) -> float:
        """Calculates the reaction distance of the go-kart.

        :return: The reaction distance in meters.
        """
        time_until_next = 1 / config["object_detection"]["max_frame_rate"]
        meters_per_second = self.speed_controller.current_speed / 3.6

        return meters_per_second * time_until_next

    def get_stopping_distance(self) -> float:
        """Calculates the stopping distance of the go-kart.

        :return: The stopping distance in meters.
        """
        return self.get_reaction_distance() + self.get_braking_distance()

    def handle(self, predictions: Boxes) -> None:
        """Handles the predictions.

        :param predictions: The predictions to handle.
        """
        if self.disabled:
            return

        for handler in self.handlers:
            if not self.lane_assist.enabled and handler.manual_mode:
                continue

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

    def set_steering(self, angle: float) -> None:
        """Set the steering of the go-kart.

        :param angle: The angle to set.
        """
        self.speed_controller.can_controller.set_steering(angle)
