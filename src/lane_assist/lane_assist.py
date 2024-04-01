import threading
import time
from collections.abc import Callable, Generator
from threading import Thread

import numpy as np

from config import config
from driving.can_controller.can_controller_interface import ICANController
from driving.speed_controller.speed_controller_interface import ISpeedController
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.line_detector import filter_lines, get_lines
from lane_assist.line_following.path_follower import PathFollower
from lane_assist.line_following.path_generator import Path, generate_driving_path
from lane_assist.stopline_assist import StoplineAssist

colours = {
    LineType.SOLID: (0, 255, 0),
    LineType.DASHED: (0, 0, 255),
}


class LaneAssist:
    """A class to add lane assist to the kart.

    This class takes a function that generates an image, a line follower class and a can controller.
    It will then generate a path to follow and follow the path using the line following class.

    The image should be in grayscale and topdown.

    :param image_generation: A function that generates images.
    :param path_follower: The line follower class.
    :param speed_controller: The speed controller.
    :param adjust_speed: A function that calculates the dynamic speed that can be driven on the generated path
    :param requested_lane: The lane to follow.
    :param stopline_assist: The stopline assist class.
    :param can_controller: The can controller.
    """

    image_generator: Callable[[], Generator[np.ndarray, None, None]]
    path_follower: PathFollower

    stopline_assist: StoplineAssist
    can_controller: ICANController
    speed_controller: ISpeedController
    adjust_speed: Callable[[Path], int]
    requested_lane: int

    def __init__(
        self,
        image_generation: Callable[[], Generator[np.ndarray, None, None]],
        path_follower: PathFollower,
        speed_controller: ISpeedController,
        adjust_speed: Callable[[Path], int] = lambda _: 1,
    ) -> None:
        """Initialize the lane assist.

        :param image_generation: A function that generates images.
        :param path_follower: The line follower class.
        :param speed_controller: The speed controller.
        :param adjust_speed: A function that calculates the dynamic speed that can be driven on the generated path
        """
        self.image_generator = image_generation
        self.adjust_speed = adjust_speed
        self.stopline_assist = StoplineAssist(speed_controller)

        self.can_controller = speed_controller.can_controller
        self.speed_controller = speed_controller

        self.path_follower = path_follower
        self.requested_lane = config.lane_assist.line_following.requested_lane

    def start(self, multithreading: bool = False) -> None | Thread:
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
        lines, stop_lines = get_lines(image)
        driving_lines = filter_lines(lines, image.shape[1] // 2)

        if len(driving_lines) < 2:
            return

        # act on the lines in the image
        self.__follow_path(driving_lines, image.shape[1] // 2, self.requested_lane)
        self.stopline_assist.handle_stoplines(stop_lines)

    def __follow_path(self, lines: list[Line], car_position: float, lane: int) -> None:
        """Follow the path.

        This function will follow the path based on the lines in the image.
        If the lane is not available, it will throw an error

        :param lines: the lines in the image.
        :param lane: The lane to follow.
        """
        # generate the driving path
        path = generate_driving_path(lines, lane)
        speed = self.adjust_speed(path)
        self.speed_controller.target_speed = speed

        # steer the kart based on the path and its position.
        steering_fraction = self.path_follower.get_steering_fraction(path.points, car_position)
        self.can_controller.set_steering(steering_fraction)