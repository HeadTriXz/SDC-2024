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


def basic_filter(img: np.ndarray, calibration: CalibrationData) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param img: The image to filter.
    :return: The filtered image and the peaks.
    """
    histogram = np.sum(img, axis=1) / 255
    zebra_height_m = config.line_detection.thresholds.zebra_crossing
    hpx = calibration.pixels_per_meter * zebra_height_m

    width = calibration.pixels_per_meter * 0.5
    peaks = scipy.signal.find_peaks(histogram, height=hpx, width=width)[0]

    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram, peaks, rel_height=config.line_detection.filtering.rel_height
    )

    histogram_peaks = list(map(lambda params: HistogramPeak(*params), zip(peaks, widths, lefts, rights)))
    margin = calibration.get_pixels(config.line_detection.filtering.margin)

    for peak in histogram_peaks:
        if peak.width > calibration.pixels_per_meter * 6:
            continue

        left = max(int(peak.left - margin), 0)
        right = min(int(peak.right + margin), img.shape[1])

        img[left:right] = 0

    return img, histogram_peaks
