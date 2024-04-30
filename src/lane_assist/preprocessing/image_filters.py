import dataclasses

import cv2
import numpy as np
import scipy

from config import config
from utils.calibration_data import CalibrationData


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


def basic_filter(img: np.ndarray, calibration: CalibrationData) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param img: The image to filter.
    :return: The filtered image and the peaks.
    """
    third = img.shape[0] // 3
    pixels = img[:, third : 2 * third]

    histogram = np.sum(pixels, axis=1)
    width = calibration.pixels_per_meter * 0.5
    peaks = scipy.signal.find_peaks(
        histogram,
        height=config.lane_assist.line_detection.thresholds.zebra_crossing,
        width=width
    )[0]

    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram,
        peaks,
        rel_height=config.lane_assist.line_detection.filtering.rel_height
    )

    histogram_peaks = list(map(lambda params: HistogramPeak(*params), zip(peaks, widths, lefts, rights)))

    for peak in histogram_peaks:
        if peak.width > calibration.pixels_per_meter * 6:
            continue

        img[int(peak.left): int(peak.right)] = 0
        img[int(peak.left): int(peak.right)] = 0

    return img, histogram_peaks


def filter_small_clusters(img: np.ndarray, min_size: int = 100) -> np.ndarray:
    """Filter clusters of pixels smaller then a certain area."""
    # Find the connected components
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)

    sizes = stats[1:, -1]
    nb_components = nb_components - 1

    for i in range(0, nb_components):
        if sizes[i] <= min_size:
            img[output == i + 1] = 0

    return img
