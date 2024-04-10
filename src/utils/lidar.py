import numpy as np
import matplotlib.pyplot as plt
from math import floor
from adafruit_rplidar import RPLidar
from sklearn.cluster import DBSCAN
import matplotlib
PORT_NAME = "COM4"
lidar = RPLidar(None, PORT_NAME, timeout=3)
max_distance = 0

def find_obstacle_distance(data, anglemin, anglemax):
    point = 15000
    for i in range(anglemin, anglemax):
        if point > data[i] > 500:
            point = data[i]
    return point

def detect_walls(data):
    """A function used to detect the walls around the car
    to the right of the car, there will be a straight line with a hole in it, detect this line
    """
    angles = np.linspace(0, 2 * np.pi, len(data), endpoint=False)
    coordinates = [(angle, distance) for angle, distance in zip(angles, data)]

    # Voer DBSCAN clustering uit op de coördinaten
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    dbscan.fit(coordinates)
    # Verzamel de clusterpunten die als muren worden beschouwd
    wall_points = [coordinates[i] for i in range(len(coordinates)) if dbscan.labels_[i] != -1]

    return wall_points


def display_minimum(data):
    #find the point on 0:40 and on 320:360
    point = min(data[0:40])
    pointleft = min(data[320:360])
    if point < pointleft:
        point = pointleft
    if point > 500:
        plt.plot(point,0, "ro")  # Plot het eerste punt in het rood
    leftpoint = min(data[45:120])
    rightpoint = min(data[225:320])
    if leftpoint > 500:
        plt.plot(0, leftpoint, "ro")
    if rightpoint > 500:
        plt.plot(0, -rightpoint, "ro")

def right_free(data) -> bool:
    """A function that checks if the right side of the car is free

    :param data: the data from the lidar
    :return: True if the right side is free, False otherwise
    """
    for i in range(20,120):
        if data[i] < 2500:
            return False

    if min(data[20:120]) < 2500:
        if data[i+1] < 2500 or data[i-1] < 2500:
            return False
    return True

def free_side(data, anglemin, anglemax) -> bool:
    """A function that checks if the right side of the car is free

    :param data: the data from the lidar
    :param side: the side to check, either "right" or "left"
    :return: True if the side is free, False otherwise
    """
    for i in range(anglemin, anglemax):
        if data[i] < 2500 and (data[i+1] < 2500 or data[i-1] < 2500):
            return False

        return True

def process_data(data):
    matplotlib.use("TkAgg")
    angles = np.linspace(0, 2*np.pi, len(data), endpoint=False)
    distances = np.array(data)
    newdistances = distances.copy()
    for i in range(len(distances) - 1):
        if distances[i] < 500:
            newdistances[i] = 15000

        if np.abs(distances[i] - distances[i + 1]) > 1500:
            newdistances[i] = np.nan

    plt.plot(find_obstacle_distance(newdistances, 0,40), 0, "ro")
    plt.plot()
    # Omzetten naar cartesische coördinaten
    x = newdistances * np.cos(angles)
    y = newdistances * np.sin(angles)
    #detect_lines(newdistances)
    plt.clf()  # Wis de plot
    plt.scatter(x, y, s=5)  # s is de grootte van de punten
    plt.title("2D Lidar")
    plt.xlabel("X")
    plt.ylabel("Y")
    display_minimum(newdistances)
    # print(right_free(newdistances))
    print(free_side(newdistances,60, 120))
    plt.xlim(-1000, 10000)
    plt.ylim(-2000,2000)
    plt.grid(True)
    plt.pause(0.01)

try:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        scan_data = [0]*360
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
except KeyboardInterrupt:
    print("Stopping.")
lidar.stop()
lidar.disconnect()
