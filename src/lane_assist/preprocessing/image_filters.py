import cv2
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


def basic_filter_ranges(image: np.ndarray, hpx: int, width: int, margin: int) -> list[HistogramPeak]:
    """Get the parts of the image to filter."""
    histogram = np.concatenate([[0], np.sum(image, axis=1) / 255, [0]])

    peaks = scipy.signal.find_peaks(histogram, height=hpx, width=width)[0]
    widths, _, lefts, rights = scipy.signal.peak_widths(
        histogram, peaks, rel_height=config["line_detection"]["filtering"]["rel_height"]
    )

    return list(
        map(
            lambda params: HistogramPeak(
                center=params[0],
                width=params[1],
                left=max(int(params[2] - margin) - 1, 0),
                right=min(int(params[3] + margin) - 1, image.shape[0]),
            ),
            zip(peaks, widths, lefts, rights),
        )
    )


def morphex_filter(image: np.ndarray, calibration: CalibrationData) -> np.ndarray:
    """Filter the image using morphological operations."""
    hpx = calibration.get_pixels(config["line_detection"]["thresholds"]["zebra_crossing"])
    margin = calibration.get_pixels(config["line_detection"]["filtering"]["margin"])
    width = calibration.get_pixels(0.5)

    _, image = cv2.threshold(image, config["preprocessing"]["filter_threshold"], 255, cv2.THRESH_BINARY)
    full_mask = np.full((image.shape[0] + 10, image.shape[1]), 255, dtype=np.uint8)
    full_mask[:-10] = image

    histogram_peaks = basic_filter_ranges(image, hpx, width, margin)
    if len(histogram_peaks) == 0:
        return np.zeros_like(image)

    full_mask = cv2.dilate(full_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 4)), iterations=1)
    full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
    full_mask = cv2.dilate(full_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)), iterations=1)

    full_mask[full_mask > 100] = 255
    full_mask[full_mask <= 100] = 0

    mask = np.zeros_like(image)
    for peak in histogram_peaks:
        mask[peak.left : peak.right] = full_mask[peak.left : peak.right]

    return mask
