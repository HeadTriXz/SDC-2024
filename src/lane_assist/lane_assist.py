import cv2
import numpy as np
import threading
import time

from collections.abc import Callable, Generator

from src.calibration.data import CalibrationData
from src.config import config
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController
from src.lane_assist.line_detection.line import Line, LineType
from src.lane_assist.line_detection.line_detector import filter_lines, get_lines
from src.lane_assist.line_following.dynamic_speed import get_max_path_speed
from src.lane_assist.line_following.path_follower import PathFollower
from src.lane_assist.line_following.path_generator import Path, generate_driving_path
from src.lane_assist.stop_line_assist import StopLineAssist
from src.telemetry.app import TelemetryServer


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
        enabled: Whether lane assist is enabled.
        image_generator: A function that generates images.
        lines: The lines on the road.
        requested_lane: The lane to follow.
        speed_controller: The speed controller.
        telemetry: The telemetry server.

    """

    can_controller: ICANController
    enabled: bool = False
    image_generator: Callable[[], Generator[np.ndarray, None, None]]
    lines: list[Line]
    requested_lane: int
    speed_controller: ISpeedController
    telemetry: TelemetryServer

    __path_follower: PathFollower
    __stop_line_assist: StopLineAssist
    __calibration: CalibrationData

    def __init__(
            self,
            image_generation: Callable[[], Generator[np.ndarray, None, None]],
            stop_line_assist: StopLineAssist,
            speed_controller: ISpeedController,
            telemetry: TelemetryServer,
            calibration: CalibrationData,
    ) -> None:
        """Initialize the lane assist.

        :param image_generation: A function that generates images.
        :param stop_line_assist: The stop line assist instance.
        :param speed_controller: The speed controller.
        :param telemetry: The telemetry server.
        :param calibration: The calibration data.
        """
        self.can_controller = speed_controller.can_controller
        self.speed_controller = speed_controller
        self.image_generator = image_generation
        self.lines = []
        self.requested_lane = config.line_following.requested_lane
        self.telemetry = telemetry

        self.__stop_line_assist = stop_line_assist
        self.__path_follower = PathFollower(calibration, speed_controller)
        self.__calibration = calibration

    def lane_assist_loop(self, image: np.ndarray) -> None:
        """Lane assist loop.

        This function will take an image and follow the path in the image.

        :param image: The image to follow the path in.
        """
        lines = get_lines(image, calibration=self.__calibration)
        filtered_lines = filter_lines(lines, image.shape[1] // 2)

        if len(filtered_lines) < 2:
            return

        self.lines = filtered_lines

        # Act on the lines in the image.
        path, target_point = self.__follow_path(filtered_lines, image.shape[1] // 2, self.requested_lane)
        self.__stop_line_assist.detect_and_handle(image, filtered_lines)

        # If telemetry is enabled, send the image to the telemetry server.
        if config.telemetry.enabled:
            rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            # Draw the path on the image.
            for i in range(len(path.points) - 1):
                cv2.line(rgb, (int(path.points[i][0]), int(path.points[i][1])),
                              (int(path.points[i + 1][0]), int(path.points[i + 1][1])), (255, 255, 0), 2)

            # Draw the lines on the image.
            for line in filtered_lines:
                for point in line.points:
                    cv2.circle(rgb, (point[0], point[1]), 3, colours[line.line_type], -1)

            # Draw the target point on the image.
            cv2.circle(rgb, (int(target_point[0]), int(target_point[1])), 4, (0, 0, 255), -1)

            # Send the image to the telemetry server.
            self.telemetry.websocket_handler.send_image("laneassist", rgb)

    def start(self, multithreading: bool = False) -> threading.Thread | None:
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
        self.enabled = not self.enabled
        self.__path_follower.reset()

    def __follow_path(self, lines: list[Line], car_position: float, lane: int) -> tuple[Path, np.ndarray]:
        """Follow the path.

        This function will follow the path based on the lines in the image.
        If the lane is not available, it will throw an error

        :param lines: The lines in the image.
        :param car_position: The position of the car in the image.
        :param lane: The lane to follow.
        :return: The path and the target point.
        """
        # Generate the driving path.
        path = generate_driving_path(self.__calibration, lines, lane)
        speed = min(self.speed_controller.max_speed, get_max_path_speed(path))
        self.speed_controller.target_speed = speed

        # Steer the kart based on the path and its position.
        target_point = self.__path_follower.get_path_point(path.points)
        steering_fraction = self.__path_follower.get_steering_fraction(target_point, car_position)

        self.can_controller.set_steering(steering_fraction)
        return path, target_point

    def __run(self) -> None:
        """Run the lane assist loop."""
        for gray_image in self.image_generator():
            if not self.enabled:
                time.sleep(0.5)
                continue

            self.lane_assist_loop(gray_image)
            time.sleep(0)
