import cv2
import numpy as np
import threading
import time

from collections.abc import Callable, Generator
from config import config
from driving.can import ICANController
from driving.speed_controller import ISpeedController
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.line_detector import filter_lines, get_lines
from lane_assist.line_following.path_follower import PathFollower
from lane_assist.line_following.path_generator import Path, generate_driving_path
from lane_assist.stopline_assist import StopLineAssist
from telemetry.app import TelemetryServer
from typing import Optional

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
        adjust_speed: A function that calculates the dynamic speed that can be driven on the generated path
        can_controller: The can controller.
        image_generator: A function that generates images.
        path_follower: The line follower class.
        requested_lane: The lane to follow.
        speed_controller: The speed controller.
        stopline_assist: The stopline assist class.
        telemetry: The telemetry server.

    """

    adjust_speed: Callable[[Path], int]
    can_controller: ICANController
    image_generator: Callable[[], Generator[np.ndarray, None, None]]
    path_follower: PathFollower
    requested_lane: int
    speed_controller: ISpeedController
    stopline_assist: StopLineAssist
    telemetry: TelemetryServer

    def __init__(
        self,
        image_generation: Callable[[], Generator[np.ndarray, None, None]],
        path_follower: PathFollower,
        speed_controller: ISpeedController,
        adjust_speed: Callable[[Path], int],
        telemetry: TelemetryServer
    ) -> None:
        """Initialize the lane assist.

        :param image_generation: A function that generates images.
        :param path_follower: The line follower class.
        :param speed_controller: The speed controller.
        :param adjust_speed: A function that calculates the dynamic speed that can be driven on the generated path.
        :param telemetry: The telemetry server.
        """
        self.adjust_speed = adjust_speed
        self.can_controller = speed_controller.can_controller
        self.image_generator = image_generation
        self.path_follower = path_follower
        self.requested_lane = config.lane_assist.line_following.requested_lane
        self.speed_controller = speed_controller
        self.stopline_assist = StopLineAssist(speed_controller)
        self.telemetry = telemetry

        self.frame_times = []

    def start(self, multithreading: bool = False) -> Optional[threading.Thread]:
        """Start the lane assist.

        This function will start the lane assist. It will generate images and follow the path.

        :param multithreading: If the lane assist should run on a separate thread.
        """
        if multithreading:  # TODO: investigate why multi threaded is extremely slow
            thread = threading.Thread(target=self.__run, daemon=True)
            thread.start()
            return thread

        return self.__run()

    def __run(self) -> None:
        for gray_image in self.image_generator():
            self.lane_assist_loop(gray_image)
            time.sleep(0)

    def lane_assist_loop(self, image: np.ndarray) -> None:
        """Lane assist loop.

        This function will take an image and follow the path in the image.

        :param image: The image to follow the path in.
        """
        start_time = time.perf_counter()

        lines, stop_lines = get_lines(image)
        driving_lines = filter_lines(lines, image.shape[1] // 2)

        # FIXME: remove telemetry
        if config.telemetry.enabled:
            rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            # Draw the lines on the image.
            for line in driving_lines:
                for point in line.points:
                    cv2.circle(rgb, (point[0], point[1]), 3, colours[line.line_type], -1)

            self.telemetry.websocket_handler.send_image("laneassist", rgb)

        if len(driving_lines) < 2:
            self.frame_times.append(time.perf_counter() - start_time)
            self.telemetry.websocket_handler.send_text("fps", f"{1 / (time.perf_counter() - start_time):.2f}")
            return

        # Act on the lines in the image.
        self.__follow_path(driving_lines, image.shape[1] // 2, self.requested_lane)
        self.stopline_assist.handle_stop_lines(stop_lines)

        # FIXME: remove telemetry
        self.frame_times.append(time.perf_counter() - start_time)
        self.telemetry.websocket_handler.send_text("fps", f"{1 / (time.perf_counter() - start_time):.2f}")
        self.telemetry.websocket_handler.send_text("error", f"{self.path_follower.errors[-1]:.2f}")

    def __follow_path(self, lines: list[Line], car_position: float, lane: int) -> None:
        """Follow the path.

        This function will follow the path based on the lines in the image.
        If the lane is not available, it will throw an error

        :param lines: the lines in the image.
        :param lane: The lane to follow.
        """
        # Generate the driving path.
        path = generate_driving_path(lines, lane)
        speed = self.adjust_speed(path)
        self.speed_controller.target_speed = speed

        # Steer the kart based on the path and its position.
        steering_fraction = self.path_follower.get_steering_fraction(path.points, car_position)
        self.can_controller.set_steering(steering_fraction)
