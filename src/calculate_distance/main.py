import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

"""Calibrated parameters"""
a0 = 325
b0 = 0.826
c0 = 386
d0 = -1.475

"""Functions"""
def formula(x, a, b, c, d):
    """The formula to fit the data to."""
    return a / (x - c) ** b + d

def recalibrate_parameters(x, y) -> list[float]:
    """Recalibrates the parameters of the formula to fit the given data."""
    popt, pcov = curve_fit(formula, x, y, p0=[a0, b0, c0, d0])
    return popt

def plot_data(x, y, popt):
    """Plots the given data and the fitted line."""
    x_reg = np.linspace(430, 1080, 100)

    plt.plot(x, y, 'ro', label='Original data')
    plt.plot(x_reg, formula(x_reg, *popt), 'b-', label='Fitted line')
    plt.legend()

    plt.show()

def y_to_meters(y: int) -> float:
    """Converts the y-value to meters."""
    if y <= c0:
        raise ValueError("The y-value is too low.")

    return formula(y, a0, b0, c0, d0)

def meters_to_y(meters: float) -> int:
    """Converts the meters value to y-value."""
    return int((a0 / (meters - d0)) ** (1 / b0) + c0)


if __name__ == '__main__':
    X = np.array([1028, 840, 733, 621, 561, 523, 499, 481, 468, 457, 449, 443])
    Y = np.array([0.1, 0.6, 1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 7.1, 8.1, 9.1, 10.1])

    popt = recalibrate_parameters(X, Y)
    plot_data(X, Y, popt)

    print("The recalibrated parameters are:")
    print(f"a = {popt[0]}")
    print(f"b = {popt[1]}")
    print(f"c = {popt[2]}")
    print(f"d = {popt[3]}")

    print("Testing the conversion functions:")
    print(f"y_to_meters(1028) = {y_to_meters(1028)}")
    print(f"y_to_meters(733) = {y_to_meters(733)}")
    print(f"y_to_meters(387) = {y_to_meters(387)}")
    print(f"meters_to_y(0.1) = {meters_to_y(0.1)}")
    print(f"meters_to_y(10.1) = {meters_to_y(10.1)}")
    print(f"meters_to_y(5000) = {meters_to_y(5000)}")
