import cv2

from lane_assist.image_manipulation.image_stitch import adjust_gamma, stitch_images
from lane_assist.line_detection import get_lines

if __name__ == "__main__":
    # Load unstitched_images
    center_img = cv2.imread("../resources/images/straight/center.jpg")
    left_img = cv2.imread("../resources/images/straight/left.jpg")
    right_img = cv2.imread("../resources/images/straight/right.jpg")

    # adjust the gamma of the unstitched_images so the bright unstitched_images are giving less false positives
    left_img = adjust_gamma(left_img, 0.62)
    right_img = adjust_gamma(right_img, 0.62)
    stitched = stitch_images(left_img, center_img, right_img)

    # get the lines of the image
    lines = get_lines(stitched)
    print(lines)  # noqa: T201
