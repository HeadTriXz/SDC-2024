import cv2
import matplotlib.pyplot as plt
import numpy as np
from simple_pid import PID

from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection import filter_lines, get_lines
from lane_assist.path_generation import generate_driving_path


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

    pid = PID(0.1, 0.01, 0.005, setpoint=0)

    # simulator settings
    max_angle_change_per_iteration = np.pi / 180 * 5

    # simulate the car driving
    bounds = (800, 900)
    pos = (450, 900)

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

        # get the target point
        target_point = None
        i = 0
        for i, point in enumerate(path):
            if point[1] < pos[1] and i + 3 < len(path):
                target_point = path[i + 3]
                break

        if target_point is None:
            print("no target point found")
            break

        # draw line to target point
        plt.plot([pos[0], target_point[0]], [pos[1], target_point[1]], "b-")

        # get the y distacne to the target point
        x_distance_to_target = target_point[0] - pos[0]

        # calculate the angle to the target point. with rotation as the current rotation of the car
        # get the angle of speed vector
        current_angle = np.arctan2(speed[1], speed[0])
        control_variable = pid(x_distance_to_target)

        angle = current_angle - control_variable

        if angle > max_angle_change_per_iteration:
            angle = max_angle_change_per_iteration
        elif angle < -max_angle_change_per_iteration:
            angle = -max_angle_change_per_iteration

        # steer into the direction of the target point
        speed = rotate_vector(speed, angle)

        # draw the path

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
