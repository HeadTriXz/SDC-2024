import dataclasses
import numpy as np
import scipy

from src.calibration.data import CalibrationData
from src.config import config


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


def basic_filter(image: np.ndarray, calibration: CalibrationData) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param image: The image to filter.
    :param calibration: The calibration data.
    :return: The filtered image and the peaks.
    """
    histogram = np.concatenate([[0], np.sum(image, axis=1) / 255, [0]])
    hpx = calibration.get_pixels(config.line_detection.thresholds.zebra_crossing)
    width = calibration.get_pixels(0.5)

    peaks = scipy.signal.find_peaks(histogram, height=hpx, width=width)[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram, peaks, rel_height=config.line_detection.filtering.rel_height
    )

    histogram_peaks = list(map(lambda params: HistogramPeak(*params), zip(peaks, widths, lefts, rights)))
    margin = calibration.get_pixels(config.line_detection.filtering.margin)

    for peak in histogram_peaks:
        if peak.width > calibration.get_pixels(6):
            continue

        min_y = max(int(peak.left - margin) - 1, 0)
        max_y = min(int(peak.right + margin) - 1, image.shape[0] - 1)

        image[min_y:max_y] = 0

    return image, histogram_peaks
