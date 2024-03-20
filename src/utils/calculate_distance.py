import logging
from typing import TypeVar

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

"""Calibrated parameters"""
a0 = 1.015988
b0 = 0.825809
c0 = 0.357420
d0 = -1.46350

T = TypeVar("T", np.ndarray, float)
POPT = (float, float, float, float)

"""Functions"""


def formula(x: T, a: float, b: float, c: float, d: float) -> T:
    """The formula to calculate the distance from the y-value."""
    return a / (x - c) ** b + d


def recalibrate_parameters(x: np.ndarray, y: np.ndarray) -> POPT:
    """Recalibrate the parameters of the formula to fit the given data."""
    fit = curve_fit(formula, x, y, p0=[a0, b0, c0, d0])
    return fit[0]


def plot_data(x: np.ndarray, y: np.ndarray, popt: POPT) -> None:
    """Plot the given data and the fitted line."""
    x_reg = np.linspace(0.4, 1, 1000)

    plt.plot(x, y, "ro", label="Original data")
    plt.plot(x_reg, formula(x_reg, *popt), "b-", label="Fitted line")
    plt.legend()

    plt.show()


def y_to_meters(y: int, height: int) -> float:
    """Convert a y-value to meters."""
    rel_y = y / height
    if rel_y <= c0:
        raise ValueError("The y-value is too low.")

    return formula(rel_y, a0, b0, c0, d0)


def meters_to_y(meters: float, height: int) -> int:
    """Convert meters to a y-value."""
    rel_y = (a0 / (meters - d0)) ** (1 / b0) + c0
    return int(round(rel_y * height))


if __name__ == "__main__":
    X = np.array([1028, 840, 733, 621, 561, 523, 499, 481, 468, 457, 449, 443]) / 1080
    Y = np.array([0, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]) + 0.115

    popt = recalibrate_parameters(X, Y)
    plot_data(X, Y, popt)

    logging.basicConfig(level=logging.INFO)
    logging.info("The recalibrated parameters are:")
    logging.info("a = %.6f", popt[0])
    logging.info("b = %.6f", popt[1])
    logging.info("c = %.6f", popt[2])
    logging.info("d = %.6f", popt[3])
