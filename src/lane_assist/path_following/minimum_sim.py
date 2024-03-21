import time

import cv2
import numpy as np
from matplotlib import pyplot as plt

from lane_assist import filter_lines, generate_driving_path, get_lines, topdown
from lane_assist.path_following import LineFollowing


def rotate_vector(vector: np.ndarray, angle: float) -> tuple[float, float]:
    """Rotate a vector by a given angle."""
    x = vector[0] * np.cos(angle) - vector[1] * np.sin(angle)
    y = vector[0] * np.sin(angle) + vector[1] * np.cos(angle)
    return x, y


def basic_simulation() -> None:
    """Simulate a car driving on a path."""
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

    line_following = LineFollowing(0.1, 0.01, 0.05, look_ahead_distance=10)

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
        steering_angle = line_following.get_steering_angle(path, pos[0])

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
