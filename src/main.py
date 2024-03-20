import cv2

from lane_assist.image_manipulation.image_stitch import adjust_gamma, stitch_images
from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection import get_lines

if __name__ == "__main__":
    # load images
    center_img = cv2.imread("../resources/images/straight/center.jpg")
    left_img = cv2.imread("../resources/images/straight/left.jpg")
    right_img = cv2.imread("../resources/images/straight/right.jpg")

    # adjust the gamma of the images so the bright unstitched_ are giving less false positives
    left_img = adjust_gamma(left_img, 0.62)
    right_img = adjust_gamma(right_img, 0.62)
    stitched = stitch_images(left_img, center_img, right_img)

    # convert image to topdown and grayscale
    graysacle = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
    top_down_img = topdown(graysacle)

    # get the lines of the image
    lines = get_lines(top_down_img)
    print(lines)  # noqa: T201
