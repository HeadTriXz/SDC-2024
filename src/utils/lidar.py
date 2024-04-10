import numpy as np
from math import floor
from adafruit_rplidar import RPLidar

PORT_NAME = "COM4"
lidar = RPLidar(None, PORT_NAME, timeout=3)
max_distance = 0

def find_obstacle_distance(data :list[int], anglemin: int, anglemax:int) -> int:
    """A function that finds the distance to the closest obstacle in a certain angle range.

    :param data: the data from the lidar
    :param anglemin: the minimum angle to check
    :param anglemax: the maximum angle to check

    :return: the distance to the closest obstacle
    """
    point = 15000
    for i in range(anglemin, anglemax):
        if point > data[i] > 500:
            point = data[i]
    return point
def right_free(data: list[int]) -> bool:
    """A function that checks if the right side of the car is free.

    :param data: the data from the lidar
    :return: True if the right side is free, False otherwise
    """
    if min(data[20:120]) < 2500:
        for i in range(20, 120):
            if data[i] < 2500 and(data[i+1] < 2500 or data[i-1] < 2500):
                return False
    return True

def free_side(data: list[int], anglemin: int, anglemax: int) -> bool:
    """A function that checks if the side between anglemin and anglemax of the car is free.

    :param data: the data from the lidar
    :param side: the side to check, either "right" or "left"
    :return: True if the side is free, False otherwise
    """
    return all(not (data[i] < 2500 and (data[i + 1] < 2500 or data[i - 1] < 2500)) for i in range(anglemin, anglemax))

def data_processing(data: list[int]) -> list[int]:
    """A function that processes the data from the lidar.

    :param data: the data from the lidar

    :return: the processed data
    """
    distances = np.array(data)
    newdistances = distances.copy()
    for i in range(len(distances) - 1):
        if distances[i] < 500:
            newdistances[i] = 15000

        if np.abs(distances[i] - distances[i + 1]) > 1500:
            newdistances[i] = np.nan
    return newdistances

try:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        scan_data = [0]*360
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        processed_data = data_processing(scan_data)

except KeyboardInterrupt:
    lidar.stop()
lidar.stop()
lidar.disconnect()
