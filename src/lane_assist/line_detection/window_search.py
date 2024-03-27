import numpy as np
import scipy

from config import config
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window

# REPLACED WITH THE VALUES IN GLOBALS
THRESHOLDS = config.lane_assist.line_detection.thresholds


def window_search(
    img: np.ndarray, window_count: int, _pixels_per_window: int = 1, window_width: int = 60
) -> list[Line]:
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

    histogram = np.sum(img[:], axis=1)
    filter_peaks = scipy.signal.find_peaks(
        histogram,
        height=THRESHOLDS.zebra_crossing,
        distance=config.lane_assist.line_detection.line_width * 4,
    )[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(histogram, filter_peaks, rel_height=0.90)
    stop_lines_y = []

    # draw the peaks on the image

    # mask out these peaks if they are wider then a line
    for left, right in zip(lefts, rights):
        if (
            right - left < (config.lane_assist.line_detection.line_width * 2)
            and left > config.lane_assist.line_detection.line_width * 2.1
            and right < img.shape[0] - config.lane_assist.line_detection.line_width * 2.1
        ):
            stop_lines_y.append(int((left + right) / 2))

        img[int(left) : int(right)] = 0

    # check if the peaks are connected to the bottom of the image
    # if they are we need to start window search above them

    start_position = img.shape[0] - 1
    if len(rights) > 0:  # check if we have any peaks
        lowest_end_index = np.argmax(rights)

        if rights[lowest_end_index] > img.shape[0] - 10:
            start_position = int(lefts[lowest_end_index])

    # create a histogram of the image to find the lines.
    # we only use the bottom half because that should be where the lines are.
    # to get the lines we will detect and get the peaks
    weights = np.linspace(0, 1, img.shape[0] // 2)
    pixels = img[img.shape[0] // 2 :]
    pixels = np.multiply(pixels, weights[:, np.newaxis])
    histogram = np.sum(pixels, axis=0)

    merged_peaks = scipy.signal.find_peaks(
        histogram, height=THRESHOLDS.line, distance=config.lane_assist.line_detection.line_width
    )[0]

    # create the windows
    window_height = img.shape[0] // window_count  # get the height of the windows based on the amount we want.

    # update the window count
    window_count = start_position // window_height

    windows = [Window(center, start_position, window_width // 2) for center in merged_peaks]

    for _ in range(window_count):
        # check which windows overlap
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
                # pass

            # set the current position of the window
            win_y_low = window.y - window_height
            win_y_high = window.y
            win_x_low = window.x - window.margin
            win_x_high = window.x + window.margin

            # check how many white pixels are in the window
            non_zero_count = np.sum(img[win_y_low:win_y_high, win_x_low:win_x_high])
            # TODO: move the y axis into the direction of the line.
            #       this will allow us to better detect corners.
            #       will only be done if needed.

            if non_zero_count > config.lane_assist.line_detection.pixels_in_window:
                coords = np.nonzero(img[win_y_low:win_y_high, win_x_low:win_x_high])
                # get the right most pixel. this is the new position of the window
                window.move(int(np.mean(coords[1])) + win_x_low, win_y_low)
            else:
                if window.found_in_previous:
                    # remove the last point
                    window.found_in_previous = False

                window.move(window.x, win_y_low, False)

    # remove all none values

    # create the lines. we first calculate how many gaps we have filtered out.
    # this is used to determine if the line is solid or dashed. we also allow
    # for 2 extra gaps jsut in case
    filtered_count = len(filter_peaks) - len(stop_lines_y)
    lines = [
        Line(np.array(window.points[:-1]), window_height, gaps_allowed=filtered_count + 2)
        for window in windows
        if len(window.points) > 5
    ]

    if len(stop_lines_y) == 0:
        return lines

    # get the y position of the stop lines
    # get the solid lines in the image
    solid_lines = [line for line in lines if line.line_type == LineType.SOLID]

    # chck if the stop lines are at a right angle to the solid lines
    if len(solid_lines) == 0:
        return lines

    closest_indexes = []
    for stop_line in stop_lines_y:
        closest_index = np.argmin([abs(line.points[0][1] - stop_line) for line in solid_lines])
        closest_indexes.append(closest_index)

    # get the angle of the points of the solid lines
    angles = [
        np.arctan2(
            line.points[closest_index - 2][1] - line.points[closest_index][1],
            line.points[closest_index][0] - line.points[closest_index - 2][0],
        )
        for line in solid_lines
    ]

    if len(angles) == 0:
        return lines

    for angle in angles:
        # check if it is in the driving direction
        if abs(angle) < 1.2:
            return lines

    # get a close point
    closest_points = [solid_lines[i].points[closest_index] for i, closest_index in enumerate(closest_indexes)]
    line = Line(np.array(closest_points), line_type=LineType.STOP)
    lines.append(line)

    return lines
