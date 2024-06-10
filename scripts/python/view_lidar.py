import matplotlib.pyplot as plt
import numpy as np

from src.utils.lidar import Lidar


def view_lidar() -> None:
    """View the lidar data."""
    lidar = Lidar()
    lidar.start()

    while True:
        angles = np.linspace(0, 2 * np.pi, 360)
        x = lidar.scan_data * np.cos(angles)
        y = lidar.scan_data * np.sin(angles)

        plt.clf()
        plt.scatter(x, y, s=2)
        plt.xlim(-10000, 10000)
        plt.ylim(-10000, 10000)
        plt.grid()
        plt.pause(0.01)


if __name__ == "__main__":
    view_lidar()
