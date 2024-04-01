from typing import Any

import cv2
import numpy as np
from matplotlib import pyplot as plt

from config import config
from lane_assist.line_detection.line import Line, LineType
from lane_assist.line_detection.window_search import window_search
from lane_assist.preprocessing.birdview import topdown


def filter_lines(lines: list[Line], starting_point: int) -> list[Line]:
    """Get the lines between the solid lines closest to each side of the starting point."""
    i = 0
    j = 0

    while i < len(lines):
        # check if we are after the starting point
        if lines[i].points[0][0] >= starting_point and lines[i].line_type == LineType.SOLID:
            # back up until we find a solid line
            j = i
            while j > 0:
                j -= 1
                if lines[j].line_type == LineType.SOLID:
                    break
            break
        i += 1

    return lines[j : i + 1]


def get_lines(image: np.ndarray) -> tuple[list[Line], list[Any]]:
    """Get the lines in the image.

    This function will take an image and return the lines in the image.
    the image should be stitched and not top down
    """
    cv2.threshold(image, config.image_manipulation.white_threshold, 255, cv2.THRESH_BINARY, image)
    return window_search(image, 110)


def __generate_tests(filepath: str) -> None:
    """Generate the tests for the line generation."""
    images_names = ["straight", "corner", "crossing", "stopline"]
    images = [
        cv2.imread(f"../../../resources/stitched_images/{image}.jpg", cv2.IMREAD_GRAYSCALE) for image in images_names
    ]
    images = [topdown(image) for image in images]

    with open(filepath, "w") as f:
        f.write("import numpy as np\nfrom lane_assist.line_detection.line import Line, LineType\n\n")
        for image in images:
            lines, stoplines = get_lines(image)

            f.write(f"{images_names.pop(0)} = [\n")
            for line in lines:
                f.write("\t" + line.as_definition() + ",\n")
            f.write("]\n")

    return


def __main() -> None:
    image_names = [
        "straight",
        "corner",
        "crossing",
        "stopline",
    ]
    images = [cv2.imread(f"../../../resources/stitched_images/{image}.jpg") for image in image_names]

    colours = {
        LineType.SOLID: (0, 255, 0),
        LineType.DASHED: (0, 0, 255),
    }

    final_images = []

    for image in images:
        td = topdown(image)
        grayscale = cv2.cvtColor(td, cv2.COLOR_BGR2GRAY)
        lines, stoplines = get_lines(grayscale)

        # draw the lines
        for line in lines:
            for point in line.points:
                cv2.circle(td, point, 4, colours[line.line_type], -1)

        for y in stoplines:
            # draw a red line at the y position of
            cv2.line(td, (0, y), (td.shape[1], y), (255, 0, 0), 2)

        final_images.append(td)

    # show all images in the same plot
    for i, image in enumerate(final_images):
        plt.subplot(2, 2, i + 1)
        plt.imshow(image)

    plt.show()


if __name__ == "__main__":
    # __generate_tests("../../../tests/line_detection/test_data.py")
    __main()
