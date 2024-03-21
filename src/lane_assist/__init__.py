import threading
from threading import Thread

import cv2

from common.camera_stream.stream import VideoStream
from common.constants import Gear
from globals import GLOBALS
from kart_control.can_controller import CANController
from lane_assist.image_manipulation.image_stitch import adjust_gamma, stitch_images
from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection import filter_lines, get_lines
from lane_assist.path_following import LineFollowing
from lane_assist.path_generation import generate_driving_path


def lane_assist(
    left_cam: VideoStream,
    center_cam: VideoStream,
    right_cam: VideoStream,
    line_following: LineFollowing,
    can_controller: CANController,
) -> Thread:
    """Follow the path based on the lines of the road.

    this function will spawn a new thread. this thread will take pictures from the cameras, convert them to topdown
    and then get the lines in the image. it will then generate a path to follow and follow the path using the line
    following class.

    Parameters
    ----------
    :param left_cam: the left camera.
    :param center_cam: the center camera.
    :param right_cam: the right camera.
    :param line_following: the line following class.
    :param can_controller: the can controller.

    """
    thread = threading.Thread(
        target=__lane_assist, args=(left_cam, center_cam, right_cam, line_following, can_controller), daemon=True
    )
    thread.start()

    return thread


def __lane_assist(
    left_cam: VideoStream,
    center_cam: VideoStream,
    right_cam: VideoStream,
    line_following: LineFollowing,
    can_controller: CANController,
) -> None:
    while left_cam.has_next() and center_cam.has_next() and right_cam.has_next():
        # take pictures from the cameras
        can_controller.set_throttle(GLOBALS["SET_SPEED"], Gear.DRIVE)

        left_image = left_cam.next()
        center_image = center_cam.next()
        right_image = right_cam.next()

        # convert to grayscale
        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        center_image = cv2.cvtColor(center_image, cv2.COLOR_BGR2GRAY)
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # adjust the gamma of the images
        if GLOBALS["GAMMA"]["ADJUST"]:
            left_image = adjust_gamma(left_image, GLOBALS["GAMMA"]["LEFT"])
            center_image = adjust_gamma(center_image, GLOBALS["GAMMA"]["CENTER"])
            right_image = adjust_gamma(right_image, GLOBALS["GAMMA"]["RIGHT"])

        # stitch the images together, convert to topdown
        stitched_image = stitch_images(left_image, center_image, right_image)
        topdown_image = topdown(stitched_image)

        # get the lines in the image
        lines = get_lines(topdown_image)
        lines = filter_lines(lines, topdown_image.shape[1] // 2)
        if len(lines) < 2:
            continue

        # get the path to drive on.
        path = generate_driving_path(lines, GLOBALS["REQUESTED_LANE"])
        # follow the path
        steering_percent = line_following.get_steering_fraction(path, GLOBALS["REQUESTED_LANE"])
        can_controller.set_steering(steering_percent)

    print("Done")
    can_controller.set_brake(100)
    can_controller.set_throttle(0, Gear.NEUTRAL)
    can_controller.set_steering(0)
