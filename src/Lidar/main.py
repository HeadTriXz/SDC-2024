import numpy as np
import matplotlib.pyplot as plt
from math import floor
from adafruit_rplidar import RPLidar
from sklearn.cluster import DBSCAN
import matplotlib
import cv2
import time


PORT_NAME = 'COM4'
lidar = RPLidar(None, PORT_NAME, timeout=3)
max_distance = 0

def find_obstacle_distance(data, anglemin, anglemax):
    point = 15000
    for i in range(anglemin, anglemax):
        if point > data[i] > 500:
            point = data[i]
    return point

def detect_walls(data):
    """a function used to detect the walls around the car
    to the right of the car, there will be a straight line with a hole in it, detect this line"""
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
        plt.plot(point,0, 'ro')  # Plot het eerste punt in het rood
    leftpoint = min(data[45:120])
    rightpoint = min(data[225:320])
    if leftpoint > 500:
        plt.plot(0, leftpoint, 'ro')
    if rightpoint > 500:
        plt.plot(0, -rightpoint, 'ro')

def process_data(data):
    #matplotlib.use('TkAgg')
    # Gegevens omzetten naar polaire coördinaten
    angles = np.linspace(0, 2*np.pi, len(data), endpoint=False)
    distances = np.array(data)
    newdistances = distances

    for i in range(len(distances)):
        if distances[i] < 500:
            newdistances[i] = 15000
        if distances[i] - distances[i-1] > 1500 or distances[i] - distances[i-1] < -1500:
            #newdistances[i] = np.nan
            newdistances[i]= np.nan
    plt.plot(find_obstacle_distance(newdistances, 0,40), 0, 'ro')
    plt.plot()
    # Omzetten naar cartesische coördinaten
    x = newdistances * np.cos(angles)
    y = newdistances * np.sin(angles)
    #detect_lines(newdistances)

    plt.clf()  # Clear het huidige plot
    plt.scatter(x, y, s=5)  # s is de grootte van de punten, je kunt deze aanpassen aan je voorkeur
    plt.title('2D Lidar')
    plt.xlabel('X')
    plt.ylabel('Y')
    display_minimum(newdistances)
    #print(find_obstacle_distance(newdistances, 0, 40))
    #print(newdistances)
    #fix the plot in place om -5 to 5
    plt.xlim(-1000, 10000)
    plt.ylim(-2000,2000)
    #print(freeside(newdistances, 0, 40, 1500))
    plt.grid(True)
    #plt.axis('equal')  # Zorgt ervoor dat de aspect ratio van de plot correct is
    plt.pause(0.1)  # Pauze om de plot te laten zien (voor realtime effect)
    # save the newdistances to a file .txt
    #dump the data to a file, split by a |
    #np.savetxt('lidar.txt', newdistances, delimiter='|')

try:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        scan_data = [0]*360
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data)  # dit is de data
except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()
