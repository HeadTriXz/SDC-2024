import cv2

from common.constants import SpeedMode


# Speed Controller
speed_mode: SpeedMode = SpeedMode.SLOW
speed_mode_to_speed = {
    SpeedMode.SLOW:      25,
    SpeedMode.MEDIUM:    50,
    SpeedMode.FAST:      75,
    SpeedMode.VERY_FAST: 100
}

speed = speed_mode_to_speed[speed_mode]

class Calibration:
    """The calibration settings for the bird's eye view perspective matrix."""

    ARUCO_DICT = cv2.aruco.DICT_4X4_50
    BOARD_HEIGHT = 8
    BOARD_WIDTH = 6
    MAX_IMAGE_WIDTH = 1920
    MAX_IMAGE_HEIGHT = 1080
    MARKER_LENGTH = 0.070
    SQUARE_LENGTH = 0.094
