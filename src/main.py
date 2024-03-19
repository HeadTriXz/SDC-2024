import cv2
from matplotlib import pyplot as plt

from lane_assist.image_manipulation.image_stitch import adjust_gamma, stitch_images
from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection import get_lines, filter_lines
from lane_assist.path_generation import generate_driving_path

if __name__ == "__main__":
    # load cameras
    cam1 = cv2.VideoCapture(0)
    cam2 = cv2.VideoCapture(1)
    cam3 = cv2.VideoCapture(2)


    # load images
    while True:
        # maken fotos
        ret1, frame1 = cam1.read()
        ret2, frame2 = cam2.read()
        ret3, frame3 = cam3.read()

        # convert to grayscale
        frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        frame3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)

        # adjust gamma
        frame1 = adjust_gamma(frame1, 0.6)
        frame2 = adjust_gamma(frame2, 0.6)
        frame3 = adjust_gamma(frame3, 0.6)

        # stitch/topdown
        stitched = stitch_images(frame1, frame2, frame3)
        topdown_image = topdown(stitched)

        # get lines
        lines = get_lines(topdown_image)
        lines = filter_lines(lines, 400)
        # generate driving path
        path = generate_driving_path(lines, 1)

        # get steering angle
        steering_angle = get_steering_angle(path, 400)


        # follow path
            # set steering
