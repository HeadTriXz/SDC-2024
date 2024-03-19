import cv2
import matplotlib.pyplot as plt
import numpy as np
from simple_pid import PID

from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection import filter_lines, get_lines
from lane_assist.path_generation import generate_driving_path

pid = PID(0.1, 0.05, 0.01, setpoint=0)


def get_steering_angle(path, x):
    if path.shape[0] == 0:
        return 0

    target_point = path[-1] if path.shape[0] < 10 else path[10]

    # get the y distacne to the target point
    x_distance_to_target = target_point[0] - x
    angle = pid(x_distance_to_target)
    print(angle, x_distance_to_target, x, target_point[0])

    return angle


def rotate_vector(vector, angle):
    x = vector[0] * np.cos(angle) - vector[1] * np.sin(angle)
    y = vector[0] * np.sin(angle) + vector[1] * np.cos(angle)
    return x, y


import time


def basic_simulation():
    stitched = cv2.imread("../../../resources/stitched_images/crossing.jpg", cv2.IMREAD_GRAYSCALE)
    # get the lines of the image
    stitched = topdown(stitched)
    lines = get_lines(stitched)

    lines = filter_lines(lines, 400)
    # draw the filtered lines
    for line in lines:
        plt.plot(line.points[:, 0], line.points[:, 1], "b")
    # get the driving path
    path = generate_driving_path(lines, 0)
    # draw the path
    plt.plot(path[:, 0], path[:, 1], "--", color="black")

    # simulator settings
    max_angle_change_per_iteration = np.deg2rad(30)

    # simulate the car driving
    bounds = (800, 900)
    pos = (400, 900)

    speed = (0, -20)
    max_iters = 1_000
    while True:
        if pos[0] < 0 or pos[0] > bounds[1] or pos[1] < 0 or pos[1] > bounds[1]:
            print("out of bounds")
            break

        max_iters -= 1
        if max_iters < 0:
            print("max iterations reached")
            break

        # get the steering angle
        current_angle = np.arctan2(speed[1], speed[0])
        steering_angle = get_steering_angle(path, pos[0])

        new_steering_angle = current_angle - steering_angle

        if new_steering_angle > max_angle_change_per_iteration:
            new_steering_angle = max_angle_change_per_iteration
        elif new_steering_angle < -max_angle_change_per_iteration:
            new_steering_angle = -max_angle_change_per_iteration

        # steer into the direction of the target point
        speed = rotate_vector(speed, new_steering_angle)
        # get the angle of the speed vector

        pos = (pos[0] + speed[0], pos[1] + speed[1])
        # print(pos, rotation)
        plt.plot(pos[0], pos[1], "ro")
        time.sleep(1 / 60)
        # update the plot
    # draw line to target point


if __name__ == "__main__":
    basic_simulation()
    plt.gca().invert_yaxis()
    plt.show()
