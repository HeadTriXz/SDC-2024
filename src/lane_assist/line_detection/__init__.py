import cv2
import numpy as np

from lane_assist.image_manipulation.top_down_transfrom import topdown
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window import Window
from lane_assist.line_detection.window_search import window_search
from src.utils.image import list_images


def get_lines(image: np.ndarray) -> list[Line]:
    """Get the lines in the image.

    This function will take an image and return the lines in the image.
    the image shoulb be stitched and not top down
    """
    white = cv2.inRange(image, 200, 255)
    return window_search(white, 110)


def main() -> None:
    """Example usage of the line detection.

    this function is used for testing the line detection algorithm.
    this is done by taking a few images and drawing the lines on the topdown image.
    """
    test_images = [
        cv2.imread("../../../tests/line_detection/images/corner.jpg", cv2.IMREAD_GRAYSCALE),
        cv2.imread("../../../tests/line_detection/images/straight.jpg", cv2.IMREAD_GRAYSCALE),
        cv2.imread("../../../tests/line_detection/images/crossing.jpg", cv2.IMREAD_GRAYSCALE),
        cv2.imread("../../../tests/line_detection/images/stopline.jpg", cv2.IMREAD_GRAYSCALE),
    ]

    colours = {
        LineType.SOLID: (255, 0, 0),  # red
        LineType.DASHED: (0, 255, 0),  # green
        LineType.STOP: (0, 0, 255),  # blue
    }

    final_images = []
    # convert the images, so we can find the lines
    for img in test_images:
        td_img = topdown(img)  # convert too topdown to draw the lines
        lines = get_lines(td_img)

        # draw the points on the topdown image
        for line in lines:
            for point in line.points:
                colour = colours[line.line_type]
                cv2.circle(td_img, (point[0], point[1]), 10, colour, -1)

        final_images.append(td_img)

    list_images(final_images, rows=4, cols=2)


if __name__ == "__main__":
    main()
