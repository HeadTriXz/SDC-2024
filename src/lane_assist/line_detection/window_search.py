import numpy as np
import scipy

from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window

LINE_WIDTH = 50
ZEBRA_CROSSING_THRESHOLD = 20000
FILTER_WIDTH = LINE_WIDTH * 4

LINE_THRESHOLD = 5000
LINE_DECAY = 1


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
    histogram = np.sum(img[:], axis=1)
    filter_peaks = scipy.signal.find_peaks(histogram, height=ZEBRA_CROSSING_THRESHOLD, distance=FILTER_WIDTH)[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(histogram, filter_peaks, rel_height=0.98)
    stop_lines_y = []

    # mask out these peaks if they are wider then a line
    for peak, width, left, right in zip(filter_peaks, widths, lefts, rights):
        if width > LINE_WIDTH:
            img[int(left) : int(right)] = 0
        else:
            stop_lines_y.append(int(peak))

    # create a histogram of the image to find the lines.
    # we only use the bottom half because that should be where the lines are.
    # to get the lines we will detect and get the peaks
    histogram = np.sum(img[img.shape[0] // 2 :], axis=0)
    merged_peaks = scipy.signal.find_peaks(histogram, height=LINE_THRESHOLD, distance=LINE_WIDTH)[0]

    # create the windows
    window_height = img.shape[0] // window_count  # get the height of the windows based on the amount we want.
    windows = [Window(center, img.shape[0], window_width // 2) for center in merged_peaks]

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
                        window.collided = True
                        other_window.collided = True
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
            # this will allow us to better detect corners

            # If you found > minpix pixels, recenter next window on the top nonzero pixels position
            if non_zero_count > pixels_per_window:
                coords = np.nonzero(img[win_y_low:win_y_high, win_x_low:win_x_high])
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
    lines = [Line(np.array(window.points), window_height, gaps_allowed=filtered_count + 2) for window in windows]

    if len(stop_lines_y) == 0:
        return lines

    # get the y position of the stop lines
    # get the solid lines in the image
    solid_lines = [line for line in lines if line.line_type == LineType.SOLID]

    # get the closest point for each solid line at the y of the stop lines
    closest_points = []
    for line in solid_lines:
        if line.points.shape[0] == 0:
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
