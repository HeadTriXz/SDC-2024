import cv2
import dataclasses
import numpy as np
import scipy
from matplotlib import pyplot as plt

from src.calibration.data import CalibrationData
from src.config import config

import sys

gettrace = getattr(sys, 'gettrace', None)
if gettrace is not None:
    debug = gettrace() is not None
else:
    debug = False


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


def basic_filter(image: np.ndarray, calibration: CalibrationData) -> tuple[np.ndarray, list[HistogramPeak]]:
    """Filter the image based on the axis.

    :param image: The image to filter.
    :param calibration: The calibration data.
    :return: The filtered image and the peaks.
    """
    hpx = calibration.get_pixels(config["line_detection"]["thresholds"]["zebra_crossing"])
    margin = calibration.get_pixels(config["line_detection"]["filtering"]["margin"])
    width = calibration.get_pixels(0.5)

    histogram_peaks = basic_filter_ranges(image, hpx, width, margin)
    for peak in histogram_peaks:
        if peak.width > calibration.get_pixels(6):
            continue

        image[peak.left:peak.right] = 0

    return image, histogram_peaks


def morphex_filter(image: np.ndarray, calibration: CalibrationData) -> np.ndarray:
    """Filter the image using morphological operations."""
    hpx = calibration.get_pixels(config["line_detection"]["thresholds"]["zebra_crossing"])
    margin = calibration.get_pixels(config["line_detection"]["filtering"]["margin"])
    width = calibration.get_pixels(0.5)

    _, image = cv2.threshold(image, config["preprocessing"]["filter_threshold"], 255, cv2.THRESH_BINARY)
    full_mask = np.full((image.shape[0] + 10, image.shape[1]), 255, dtype=np.uint8)
    full_mask[:-10] = image


    # has been moved, so we can use the mask to get the peaks
    histogram_peaks = basic_filter_ranges(image, hpx, width, margin)

    # this dilation has been added
    full_mask = cv2.dilate(full_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 4)), iterations=1)

    cv2.imshow("full_mask", full_mask)

    full_mask = cv2.morphologyEx(
        full_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    )

    full_mask = cv2.dilate(full_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)), iterations=1)

    full_mask[full_mask > 100] = 255
    full_mask[full_mask <= 100] = 0

    mask = np.zeros_like(full_mask)
    for peak in histogram_peaks:
        mask[peak.left:peak.right] = full_mask[peak.left:peak.right]

    mask[mask > 100] = 255
    mask[mask <= 100] = 0

    # if there are no peaks we can return the image as is
    if len(histogram_peaks) == 0:
        return np.zeros_like(image)

    return mask[:-10]


def combined_filter(gray: np.ndarray, calibration: CalibrationData) -> np.ndarray:
    """Filter the image using morphological operations."""
    # convert to grayscale

    # gray[gray == 0] = 255
    _, thresholded = cv2.threshold(gray, config["preprocessing"]["white_threshold"], 255, cv2.THRESH_BINARY)

    filter_mask = cv2.bitwise_not(morphex_filter(gray, calibration))
    filtered = cv2.bitwise_and(thresholded, filter_mask)

    # the following line can be removed but gives artifacts at the top of the
    # triangles at the bottom of the img. This will increase fps by a small
    # margin (736 -> 762 avgs of 10 runs)
    # filtered[gray == 255] = 0

    cv2.imshow("filter_mask", filter_mask)
    cv2.imshow("frame", gray)
    cv2.imshow("thresholded", thresholded)
    cv2.imshow("filtered", filtered)
    return filtered
