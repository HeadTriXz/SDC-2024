import dataclasses

import numpy as np
import scipy

from config import config


@dataclasses.dataclass
class HistogramPeak:
    """A class to represent a peak in the histogram.

    Attributes
    ----------
        center: The center of the peak.
        width: The width of the peak.
        left: The left side of the peak.
        right: The right side of the peak.

    """

    center: int
    width: int
    left: int
    right: int


def basic_filter(img: np.ndarray) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param img: The image to filter.
    :return: The filtered image and the peaks.
    """
    third = img.shape[0] // 3
    pixels = img[:, third : 2 * third]

    histogram = np.sum(pixels, axis=1)

    peaks = scipy.signal.find_peaks(
        histogram, height=config.lane_assist.line_detection.thresholds.zebra_crossing, distance=40
    )[0]

    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram,
        peaks,
        rel_height=config.lane_assist.line_detection.filtering.rel_height
    )

    histogram_peaks = list(map(lambda params: HistogramPeak(*params), zip(peaks, widths, lefts, rights)))

    if len(peaks) == 0:
        return img, []

    std = np.std(histogram)
    for peak in histogram_peaks:
        if peak.width > std:
            img[int(peak.left) : int(peak.right)] = 0
            img[int(peak.left) : int(peak.right)] = 0

    return img, histogram_peaks

