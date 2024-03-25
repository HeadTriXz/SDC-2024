import threading
import time
from collections.abc import Callable, Generator
from threading import Thread

import cv2
import numpy as np

import config
from driving.speed_controller import SpeedController, SpeedControllerState
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.line_detector import filter_lines, get_lines
from lane_assist.line_following.path_follower import PathFollower
from lane_assist.line_following.path_generator import generate_driving_path
from lane_assist.preprocessing.birdview import topdown
from lane_assist.preprocessing.stitching import adjust_gamma, stitch_images


class LaneAssist:
    """A class to add lane assist to the kart.

    This class takes a function that generates an image, a line follower class and a can controller.
    It will then generate a path to follow and follow the path using the line following class.

    The image should be in grayscale and topdown.

    Attributes
    ----------
    :param image_generation: A function that generates images.
    :param path_follower: The line follower class.
    :param speed_controller: The speed controller.
    :param adjust_speed: A function that calculates the dynamic speed that can be driven on the generated path.

    """

    def __init__(
        self,
        image_generation: Generator[np.ndarray, None, None],
        path_follower: PathFollower,
        speed_controller: SpeedController,
        adjust_speed: Callable[[np.ndarray], int] = lambda _: 100,
    ) -> None:
        """Initialize the lane assist."""
        # functions
        self.image_generator = image_generation
        self.adjust_speed = adjust_speed

        # kart controllers
        self.can_controller = speed_controller.can_controller
        self.speed_controller = speed_controller

        # remaining
        self.path_follower = path_follower

    def start(self, multithreading: bool = False) -> None | Thread:
        """Start the lane assist.

        This function will start the lane assist. It will generate images and follow the path.

        Parameters
        ----------
        :param multithreading: if the lane assist should run on a separate thread.

        """
        if multithreading:  # TODO: investigate why multi threaded is extremely slow
            thread = threading.Thread(target=self.__run, daemon=True)
            thread.start()
            return thread
        # if we want to run on the current thread, we can just call the run function
        return self.__run()

    def __run(self) -> None:
        for image in self.image_generator:
            # get the lines in the image and split them

            lines = get_lines(image)
            driving_lines = filter_lines(lines, image.shape[1] // 2)
            stop_lines = list(filter(lambda line: line.line_type == LineType.STOP, lines))

            # draw lines on the image
            for line in lines:
                for point in line.points:
                    cv2.circle(image, point, 3, (255, 0, 0), -1)

            if len(driving_lines) < 2:
                continue

            # act on the lines in the image
            self.__follow_path(
                driving_lines, image.shape[1] // 2, config.requested_lane
            )  # TODO: make requested lane dynamic
            self.__handle_stoplines(stop_lines)

            time.sleep(0)

    def __handle_stoplines(self, stoplines: list[Line]) -> None:
        if not stoplines or len(stoplines) == 0:
            return

        # if we see a stopline and the state is waiting to stop, set the state to stopped.
        # this will cause the car to stop.
        if self.speed_controller.state == SpeedControllerState.WAITING_TO_STOP:
            # TODO: take the distance to the stopline and speed into account
            self.speed_controller.state = SpeedControllerState.STOPPED

    def __follow_path(self, lines: list[Line], car_position: float, lane: int) -> None:
        """Follow the path.

        This function will follow the path based on the lines in the image.
        If the lane is not available, it will throw an error

        Parameters
        ----------
        :param lines: the lines in the image.
        :param lane: The lane to follow.

        """
        # if there are less than 2 lines, we can't create a path
        if len(lines) < 2:
            return

        # generate the driving path
        path = generate_driving_path(lines, lane)

        # adjust the speed based on the path
        speed = self.adjust_speed(path)
        self.speed_controller.target_speed = speed

        # steer the kart based on the path and its position.
        steering_fraction = self.path_follower.get_steering_fraction(path, car_position)
        self.can_controller.set_steering(steering_fraction)