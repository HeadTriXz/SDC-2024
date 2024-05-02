import cv2
import numpy as np
import threading
import time

from collections.abc import Callable, Generator
from typing import Optional

from src.config import config
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.line_detector import filter_lines, get_lines
from src.lane_assist.line_following.dynamic_speed import get_max_path_speed
from src.lane_assist.line_following.path_follower import PathFollower
from src.lane_assist.line_following.path_generator import generate_driving_path
from src.lane_assist.stopline_assist import StopLineAssist
from src.telemetry.app import TelemetryServer
from src.utils.calibration_data import CalibrationData


colours = {
    LineType.SOLID: (0, 255, 0),
    LineType.DASHED: (0, 0, 255)
}


class LaneAssist:
    """A class to add lane assist to the kart.

    This class takes a function that generates an image, a line follower class and a can controller.
    It will then generate a path to follow and follow the path using the line following class.

    The image should be in grayscale and topdown.

    Attributes
    ----------
        can_controller: The can controller.
        image_generator: A function that generates images.
        lines: The lines on the road.
        path_follower: The line follower class.
        requested_lane: The lane to follow.
        speed_controller: The speed controller.
        stopline_assist: The stopline assist class.
        telemetry: The telemetry server.

    """

    can_controller: ICANController
    image_generator: Callable[[], Generator[np.ndarray, None, None]]
    lines: list[Line]
    path_follower: PathFollower
    requested_lane: int
    speed_controller: ISpeedController
    stopline_assist: StopLineAssist
    telemetry: TelemetryServer

    __calibration: CalibrationData
    __running: bool = False

    def __init__(
            self,
            image_generation: Callable[[], Generator[np.ndarray, None, None]],
            stopline_assist: StopLineAssist,
            path_follower: PathFollower,
            speed_controller: ISpeedController,
            telemetry: TelemetryServer,
            calibration: CalibrationData,
    ) -> None:
        """Initialize the lane assist.

        :param image_generation: A function that generates images.
        :param stopline_assist: The stopline assist instance.
        :param path_follower: The line follower class.
        :param speed_controller: The speed controller.
        :param telemetry: The telemetry server.
        :param calibration: The calibration data.
        """
        self.can_controller = speed_controller.can_controller
        self.image_generator = image_generation
        self.lines = []
        self.path_follower = path_follower
        self.requested_lane = config.lane_assist.line_following.requested_lane
        self.speed_controller = speed_controller
        self.stopline_assist = stopline_assist
        self.telemetry = telemetry

        self.__calibration = calibration

    def lane_assist_loop(self, image: np.ndarray) -> None:
        """Lane assist loop.

        This function will take an image and follow the path in the image.

        :param image: The image to follow the path in.
        """
        lines = get_lines(image, calibration=self.__calibration)
        filtered_lines = filter_lines(lines, image.shape[1] // 2)

        # FIXME: remove telemetry
        if config.telemetry.enabled:
            rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            # Draw the lines on the image.
            for line in filtered_lines:
                for point in line.points:
                    cv2.circle(rgb, (point[0], point[1]), 3, colours[line.line_type], -1)

            self.telemetry.websocket_handler.send_image("laneassist", rgb)

        if len(filtered_lines) < 2:
            return

        # Act on the lines in the image.
        self.__follow_path(filtered_lines, image.shape[1] // 2, self.requested_lane)

        self.lines = filtered_lines
        self.stopline_assist.detect_and_handle(image, filtered_lines)

    def start(self, multithreading: bool = False) -> Optional[threading.Thread]:
        """Start the lane assist.

        This function will start the lane assist. It will generate images and follow the path.

        :param multithreading: If the lane assist should run on a separate thread.
        """
        if multithreading:
            thread = threading.Thread(target=self.__run, daemon=True)
            thread.start()
            return thread

        return self.__run()

    def toggle(self) -> None:
        """Toggle the lane assist."""
        self.__running = not self.__running

    def __follow_path(self, lines: list[Line], car_position: float, lane: int) -> None:
        """Follow the path.

        This function will follow the path based on the lines in the image.
        If the lane is not available, it will throw an error

        :param lines: the lines in the image.
        :param lane: The lane to follow.
        """
        # Generate the driving path.
        path = generate_driving_path(self.__calibration, lines, lane)
        speed = min(self.speed_controller.max_speed, get_max_path_speed(path))
        self.speed_controller.target_speed = speed

        # Steer the kart based on the path and its position.
        steering_fraction = self.path_follower.get_steering_fraction(path.points, car_position)
        self.can_controller.set_steering(steering_fraction)

    def __run(self) -> None:
        """Run the lane assist loop."""
        for gray_image in self.image_generator():
            if not self.__running:
                time.sleep(0.5)
                continue

            self.lane_assist_loop(gray_image)
            time.sleep(0)
