from enum import IntEnum

import cv2
import numpy as np
import scipy

from globals import GLOBALS
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window

# REPLACED WITH THE VALUES IN GLOBALS
# LINE_WIDTH = 50
# ZEBRA_CROSSING_THRESHOLD = 20000
# FILTER_WIDTH = LINE_WIDTH * 4
#
# LINE_THRESHOLD = 2500


def window_search(img: np.ndarray, window_count: int, pixels_per_window: int = 1, window_width: int = 60) -> list[Line]:
    """Get the lines in the image using the sliding window algorithm.

    first we take a histogram of the x axis. this is done to filter pout the zebra crossings.
    after that we take a histogram of the other axis. this is used for detecting the starting points of the line.
    after that we detect the 2 lines closest to the center. this will be the starting point for the windows.
    if we detect a couple od

    Todo:
    ----
    - add support for multiple horizontal lines
    - improve support for stop lines
        - fix position of stop line

    Parameter
    ---------
    :param window_width: the width of the window to use for the window search
    :param pixels_per_window: the minimum amount of pixels needed in the window to be considered part of a line
    :parameter img: the image to get the lines from.
    :parameter window_count: the amount of windows to check in the image

    """
    # take a histogram over the horizontal pixels.
    # this is used to filter out the zebra crossing.
    gl = GLOBALS["LANE_DETECTION"]

    histogram = np.sum(img[:], axis=1)
    filter_peaks = scipy.signal.find_peaks(
        histogram, height=gl["ZEBRA_CROSSING_THRESHOLD"], distance=gl["LINE_WIDTH"] * 4
    )[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(histogram, filter_peaks, rel_height=0.98)
    stop_lines_y = []

    # mask out these peaks if they are wider then a line
    for peak, width, left, right in zip(filter_peaks, widths, lefts, rights):
        if width > gl["LINE_WIDTH"]:
            img[int(left) : int(right)] = 0
        else:
            stop_lines_y.append(int(peak))

    # create a histogram of the image to find the lines.
    # we only use the bottom half because that should be where the lines are.
    # to get the lines we will detect and get the peaks
    histogram = np.sum(img[img.shape[0] // 2 :], axis=0)
    merged_peaks = scipy.signal.find_peaks(histogram, height=gl["LINE_THRESHOLD"], distance=gl["LINE_WIDTH"])[0]

    # create the windows
    window_height = img.shape[0] // window_count  # get the height of the windows based on the amount we want.
    windows = [Window(center, img.shape[0], window_width // 2) for center in merged_peaks]

    total_pixels_per_window = window_height * window_width

    for _ in range(window_count):
        # check which windows overlap
        # time the window search
        for window in windows:
            if window.collided:
                continue
            for other_window in windows:
                if window == other_window:
                    continue
                if (
                    window.x - window.margin < other_window.x + other_window.margin
                    and window.x + window.margin > other_window.x - other_window.margin
                ):
                    if window.found_in_previous and other_window.found_in_previous:
                        # kill the one furthest from the center
                        if abs(window.x - img.shape[1] // 2) < abs(other_window.x - img.shape[1] // 2):
                            other_window.collided = True
                        else:
                            window.collided = True

                    elif window.found_in_previous:
                        other_window.collided = True
                    elif other_window.found_in_previous:
                        window.collided = True

        # check if any of the collided windows have found a line in the previous section
        # if they have not, we can assume that the line has finished

        for window in windows:
            if window.collided:
                continue

            # set the current position of the window
            win_y_low = window.y - window_height
            win_y_high = window.y
            win_x_low = window.x - window.margin
            win_x_high = window.x + window.margin

            # check how many white pixels are in the window
            non_zero_count = np.count_nonzero(img[win_y_low:win_y_high, win_x_low:win_x_high])

            # TODO: move the y axis into the direction of the line.
            #       this will allow us to better detect corners

            if non_zero_count > gl["PIXELS_IN_WINDOW"]:
                coords = np.nonzero(img[win_y_low:win_y_high, win_x_low:win_x_high])
                # get the right most pixel. this is the new position of the window
                window.move(int(np.mean(coords[1])) + win_x_low, win_y_low)
            else:
                if window.found_in_previous:
                    # remove the last point
                    window.points = window.points[:-1]
                    window.found_in_previous = False

                window.move(window.x, win_y_low, False)

    # remove all none values

    # create the lines. we first calculate how many gaps we have filtered out.
    # this is used to determine if the line is solid or dashed. we also allow
    # for 2 extra gaps jsut in case
    filtered_count = len(filter_peaks) - len(stop_lines_y)
    lines = [
        Line(np.array(window.points), window_height, gaps_allowed=filtered_count + 2)
        for window in windows
        if len(window.points) > 5
    ]

    if len(stop_lines_y) == 0:
        return lines

    # get the y position of the stop lines
    # get the solid lines in the image
    solid_lines = [line for line in lines if line.line_type == LineType.SOLID]

    # get the closest point for each solid line at the y of the stop lines
    closest_points = []
    for line in solid_lines:
        if line.points.shape[0] == 0 or len(stop_lines_y) > 1:
            continue
        closest_point = line.points[np.argmin(np.abs(line.points[:, 1] - stop_lines_y))]
        closest_points.append(closest_point)

    if len(closest_points) == 0:
        return lines

    # get the distance between the closest points and the stop lines
    distances = closest_points[-1][0] - closest_points[0][0]

    # generate points in between them with a gap of window height
    points = np.array(
        [(closest_points[0][0] + i, stop_lines_y[0]) for i in range(0, distances, window_height)]
        + [(closest_points[-1][0], stop_lines_y[0])]
    )
    lines.append(Line(points, window_height, LineType.STOP))
    return lines


class Directions(IntEnum):
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3


def get_directions(img: np.ndarray, position) -> list[Directions]:
    directions = []

    # check which directions there are pixels
    if img[position[1], position[0] - 1] == 255:
        directions.append(Directions.LEFT)
    if img[position[1], position[0] + 1] == 255:
        directions.append(Directions.RIGHT)
    if img[position[1] - 1, position[0]] == 255:
        directions.append(Directions.UP)
    if img[position[1] + 1, position[0]] == 255:
        directions.append(Directions.DOWN)
    return directions


def get_oposite_direction(direction: Directions) -> Directions:
    if direction == Directions.LEFT:
        return Directions.RIGHT
    if direction == Directions.RIGHT:
        return Directions.LEFT
    if direction == Directions.UP:
        return Directions.DOWN
    if direction == Directions.DOWN:
        return Directions.UP


def is_line_end(directions, prev_direction) -> bool:
    if len(directions) == 1 and prev_direction is not None and directions[0] == get_oposite_direction(prev_direction):
        return True
    return False


def flood_fill_search(img: np.ndarray):
    """Get the lines in the image using the flood fill algorithm.

    This function will take an image and return the lines in the image.
    the image should be stitched and not top down
    """
    # convert image to whitescale
    white = cv2.inRange(img, 200, 255)

    # start from the bottom left corner and move left until we find a white pixel
    # this is the starting point of the line
    start = (0, white.shape[0] - 1)
    while white[start[1], start[0]] != 255:
        start = (start[0] + 1, start[1])

    lines = []
    prev_direction = None
    while True:
        # get the directions of the current pixel
        directions = get_directions(white, start)

        # if there are no directions, something has gone wrong and we should stop
        if len(directions) == 0:
            break

        # if there is only one direciton and it is the opoosite of the previous direction.
        # we can assume that the line has ended
        if is_line_end(directions, prev_direction):
            break

        # keep track of the previous direction
        prev_direction = directions[0] if len(directions) == 1 else None
