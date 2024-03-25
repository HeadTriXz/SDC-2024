import cv2
import matplotlib.pyplot as plt
import numpy as np

from lane_assist.line_detection.line import Line
from lane_assist.line_detection.line_detector import filter_lines, get_lines
from lane_assist.preprocessing.birdview import topdown


def generate_driving_path(lines: list[Line], requested_lane: int) -> np.ndarray:
    """Generate the driving path based on the lines.

    This function will take the lines and generate a driving path based on the lines.

    :param requested_lane: the lane we want to drive in
    :param lines: the lines to generate the path from
    :return: the driving path
    """
    lanes = [[lines[i], lines[i - 1]] for i in range(len(lines) - 1, 0, -1)]

    if requested_lane >= len(lanes):
        raise ValueError(f'Requested lane "{requested_lane}" does not exist. lanes: {len(lanes)}')

    # get the lines of the lane we want to drive in.
    a1 = lanes[requested_lane][0]
    a2 = lanes[requested_lane][1]

    # interpolate the lines so we have the same amount adn enough points.
    inter_line_points = min(len(a1.points), len(a2.points)) * 4

    new_a1_x, new_a1_y = interpolate_line(a1, inter_line_points)
    new_a2_x, new_a2_y = interpolate_line(a2, inter_line_points)

    # generate a centerline based on the two lines
    midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(inter_line_points)]
    midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(inter_line_points)]
    return np.array([midx, midy]).T


def interpolate_line(line: Line, points: int) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate the line to the given points.

    This function will interpolate the line to the given points.

    :param line: the line to interpolate
    :param points: the points to interpolate to
    :return: the interpolated line
    """
    new_y = np.linspace(line.points[0, 1], line.points[-1, 1], points)
    new_x = np.interp(new_y, line.points[::-1, 1], line.points[::-1, 0])

    return new_x, new_y


def generate_tests(filename: str) -> None:
    """Generate the tests for the line generation."""
    images_names = ["straight", "corner", "crossing", "stopline"]
    images = [
        cv2.imread(f"../../../resources/stitched_images/{image}.jpg", cv2.IMREAD_GRAYSCALE) for image in images_names
    ]
    images = [topdown(image) for image in images]

    with open(filename, "w") as f:
        f.write("import numpy as np\n\n")

        for i, image in enumerate(images):
            lines = get_lines(image)
            filtered = filter_lines(lines, 400)
            if images_names[i] == "crossing":
                path = generate_driving_path(filtered, 1)
                f.write(f"{images_names[i].upper()}_LANE_1 = np.array({path.tolist()})\n")

                path = generate_driving_path(filtered, 0)
                f.write(f"{images_names[i].upper()}_LANE_0 = np.array({path.tolist()})\n")

            else:
                path = generate_driving_path(filtered, 0)
                f.write(f"{images_names[i].upper()} = np.array({path.tolist()})\n")


def main() -> None:
    """Line detection example."""
    images_names = ["straight", "corner", "crossing", "stopline"]
    images = [cv2.imread(f"../../../resources/stitched_images/{image}.jpg") for image in images_names]
    images = [topdown(image) for image in images]
    grayscale = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]

    for i, image in enumerate(grayscale):
        lines = get_lines(image)
        filtered = filter_lines(lines, 400)
        for line in filtered:
            for point in line.points:
                cv2.circle(images[i], (point[0], point[1]), 5, (0, 255, 0), -1)

        path = generate_driving_path(filtered, 0)

        # plot the centerline
        plt.plot(path[:, 0], path[:, 1], "r")

        plt.imshow(images[i])
        plt.show()


if __name__ == "__main__":
    generate_tests("../../../tests/line_generation/test_data.py")
    main()
