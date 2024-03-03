import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

"""Calibrated parameters"""
a0 = 1.015988
b0 = 0.825809
c0 = 0.357420
d0 = -1.46350

"""Functions"""
def formula(x, a, b, c, d):
    """The formula to fit the data to."""
    return a / (x - c) ** b + d

def recalibrate_parameters(x: np.ndarray, y: np.ndarray) -> (float, float, float, float):
    """Recalibrates the parameters of the formula to fit the given data."""
    fit = curve_fit(formula, x, y, p0=[a0, b0, c0, d0])
    return fit[0]

def plot_data(x: np.ndarray, y: np.ndarray, popt: (float, float, float, float)) -> None:
    """Plots the given data and the fitted line."""
    x_reg = np.linspace(0.4, 1, 1000)

    plt.plot(x, y, 'ro', label='Original data')
    plt.plot(x_reg, formula(x_reg, *popt), 'b-', label='Fitted line')
    plt.legend()

    plt.show()

def y_to_meters(y: int, height: int) -> float:
    """Converts the y-value to meters."""
    rel_y = y / height
    if rel_y <= c0:
        raise ValueError("The y-value is too low.")

    return formula(rel_y, a0, b0, c0, d0)

def meters_to_y(meters: float, height: int) -> int:
    """Converts the meters value to y-value."""
    rel_y = ((a0 / (meters - d0)) ** (1 / b0) + c0)
    return int(round(rel_y * height))

if __name__ == '__main__':
    X = np.array([1028, 840, 733, 621, 561, 523, 499, 481, 468, 457, 449, 443]) / 1080
    Y = np.array([0, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]) + 0.115

    popt = recalibrate_parameters(X, Y)
    plot_data(X, Y, popt)

    print("The recalibrated parameters are:")
    print(f"a = {popt[0]:.6f}")
    print(f"b = {popt[1]:.6f}")
    print(f"c = {popt[2]:.6f}")
    print(f"d = {popt[3]:.6f}")

    """Ensure that the height doesn't change the result."""
    assert y_to_meters(1080, 1080) == y_to_meters(720, 720)
    assert y_to_meters(561, 1080) == y_to_meters(374, 720)

    """Ensure that y_to_meters and meters_to_y are inverse functions."""
    assert meters_to_y(y_to_meters(1080, 1080), 1080) == 1080
    assert meters_to_y(y_to_meters(720, 720), 720) == 720
    assert meters_to_y(y_to_meters(561, 1080), 1080) == 561
