import numpy as np

from lane_assist.line_detection import Line, Window
from lane_assist.line_detection.line import LineType


LINE_WIDTH = 90
ZEBRA_CROSSING_THRESHOLD = 20000
FILTER_DECAY = 10

LINE_THRESHOLD = 5000
LINE_DECAY = 1


def get_histogram_peaks(histogram: np.ndarray, peak_minimum: int, decay: int = 1) -> list[list[int]]:
    """Get the peaks in the histogram.

    get all complete peaks in the histogram.
    this is done by iterating over the histogram until the peak_minimum is reached.
    after this it will traverse the peak in both sides to find the edges of it.

    # TODO: improve detection of corners
    # TODO: support multiple stoplines

    Parameters
    ----------
    :param histogram: the histogram to extract the peaks from
    :param peak_minimum: the minimum value to be considered a peak
    :param decay: the amount of times the value needs to be lower to say it is the end of the peak

    """
    peaks = []
    # loop over all points in histogram
    index = 0
    while index < len(histogram):
        # get beginning above the threshold
        if histogram[index] > peak_minimum:
            # iterate backwards till the value is increasing.
            # this index will be the start of the peak.
            current_backward_index = index
            lower_count = 0
            while current_backward_index > 0:
                if (
                    histogram[current_backward_index] < histogram[current_backward_index - 1]
                    or histogram[current_backward_index] < 50
                ):
                    lower_count += 1

                if lower_count > decay:
                    break

                current_backward_index -= 1
            start = current_backward_index

            # iterate forwards till the value is increasing.
            # this index will be the end. we will only count it the end if we find 5 decreasing values.
            current_forward_index = index
            lower_count = 0
            while current_forward_index < len(histogram) - 1:
                if (
                    histogram[current_forward_index] < histogram[current_forward_index + 1]
                    or histogram[current_backward_index] < 50
                ) and histogram[current_forward_index] < peak_minimum:
                    lower_count += 1

                if lower_count > decay:
                    break

                current_forward_index += 1

            # add the peak to the list
            peaks.append([start, current_forward_index])

            # set the index to the end of the peak
            index = current_forward_index + 1
        else:
            index += 1

    return peaks


def merge_peaks(peaks: list, min_distance: int) -> list:
    """Merge the peaks of the histogram.

    this function will merge the peaks of the histogram if they are close to each other.

    Parameters
    ----------
    :param peaks: the peaks to merge
    :param min_distance: the minimum distance between the peaks

    """
    merged_peaks = []
    index = 0
    while index < len(peaks) - 1:
        peak = peaks[index]
        other = peaks[index + 1]
        index += 1

        peak_position = peak

        if abs(peak - other) < min_distance:
            peak_position = (peak + other) // 2
            index += 1

        merged_peaks.append(peak_position)

    if index == len(peaks) - 1:
        merged_peaks.append(peaks[-1])

    return merged_peaks




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
    avg = ZEBRA_CROSSING_THRESHOLD
    filter_peaks = get_histogram_peaks(histogram, avg, FILTER_DECAY)
    peak_widths = [peak[1] - peak[0] for peak in filter_peaks]

    stop_lines_y = []

    # mask out these peaks if they are wider then a line
    for width, peak in zip(peak_widths, filter_peaks):
        if width > LINE_WIDTH:
            img[peak[0] : peak[1]] = 0
        else:
            stop_lines_y.append(peak[len(peak) // 2])

    # create a histogram of the image to find the lines.
    # we only use the bottom half because that should eb where the lines are
    histogram = np.sum(img[img.shape[0] // 2 :], axis=0)

    # get the center of the peaks. merge if they are to close to each other.
    # if the peaks are to close to each other they will instantly kill each other.
    # the distance between these peaks most likely means that they are part of
    # the same line which has a large a gap in it.
    peak_limits = get_histogram_peaks(histogram, LINE_THRESHOLD, LINE_DECAY)
    peak_centers = [peak[0] + np.argmax(histogram[peak[0] : peak[1]]) for peak in peak_limits]
    merged_peaks = merge_peaks(peak_centers, window_width)

    # create the windows
    window_height = img.shape[0] // window_count  # get the height of the windows based on the amount we want.
    windows = [Window(center, img.shape[0] - window_height, window_width // 2) for center in merged_peaks]
    line_points = [[] for _ in range(len(windows))]
    for _ in range(window_count):
        # check which windows overlap
        overlapped_windows = []

        for i, window in enumerate(windows):
            if window.collided:
                continue
            for k, other_window in enumerate(windows):
                if window == other_window:
                    continue
                if (
                    window.x - window.margin < other_window.x + other_window.margin
                    and window.x + window.margin > other_window.x - other_window.margin
                ):
                    overlapped_windows.append((i, k))
                    break

        # check if any of the collided windows have found a line in the previous section
        # if they have not, we can assume that the line has finished
        for i, k in overlapped_windows:
            # if they don't we stop the window
            if not windows[i].found_in_previous and not windows[k].found_in_previous:
                windows[i].collided = True
                windows[k].collided = True

            if windows[i].found_in_previous:
                windows[k].collided = True
            if windows[k].found_in_previous:
                windows[i].collided = True

        for i, window in enumerate(windows):
            if window.collided:
                continue

            # set the current position of the window
            win_y_low = window.y - window_height
            win_y_high = window.y
            win_x_low = window.x - window.margin
            win_x_high = window.x + window.margin

            # check how many white pixels are in the window
            coords = np.argwhere(img[win_y_low:win_y_high, win_x_low:win_x_high] >= 100)

            # If you found > minpix pixels, recenter next window on the top nonzero pixels position
            if len(coords) > pixels_per_window:
                window.x = int(np.mean(coords[:, 1])) + win_x_low
                line_points[i].append([window.x, window.y])
                window.found_in_previous = True
            else:
                window.found_in_previous = False

            window.y = win_y_low

    # create the lines. we first calculate how many gaps we have filtered out.
    # this is used to determine if the line is solid or dashed. we also allow
    # for 2 extra gaps jsut in case
    filtered_count = len(filter_peaks) - len(stop_lines_y)
    lines = [Line(np.array(points), window_height, gaps_allowed=filtered_count + 2) for points in line_points]

    if len(stop_lines_y) == 0:
        return lines

    # get the y position of the stop lines
    # get the solid lines in the image
    solid_lines = [line for line in lines if line.line_type == LineType.SOLID]

    # get the closest point for each solid line at the y of the stop lines
    closest_points = []
    for line in solid_lines:
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
