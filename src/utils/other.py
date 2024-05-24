import numpy as np
import scipy


Coordinate = tuple[int, int] | np.ndarray


def center_of_masses(image: np.ndarray, target: int, min_pixels: int = 1) -> tuple[int, int] | None:
    """Get the center of the nearest cluster of pixels.

    :param image: The image to process.
    :param target: The x-coordinate to target (nearest to us).
    :param min_pixels: The minimum number of pixels in the cluster.
    :return: The center of the nearest cluster of pixels (x, y).
    """
    labels, num_features = scipy.ndimage.label(image)

    nearest = None
    for cluster_id in range(1, num_features + 1):
        indices = np.argwhere(labels == cluster_id)
        if len(indices) < min_pixels:
            continue

        center = indices.mean(axis=0)
        if nearest is None or abs(center[1] - target) < abs(nearest[1] - target):
            nearest = center

    if nearest is None:
        return None

    return int(nearest[1]), int(nearest[0])


def euclidean_distance(p1: Coordinate, p2: Coordinate) -> float:
    """Calculate the Euclidean distance between two points.

    :param p1: The first point.
    :param p2: The second point.
    :return: The Euclidean distance between the two points.
    """
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def find_intersection(
        line1: tuple[Coordinate, Coordinate],
        line2: tuple[Coordinate, Coordinate],
        segments: bool = True
) -> Coordinate | None:
    """Find the intersection between two lines.

    :param line1: The first line.
    :param line2: The second line.
    :param segments: Whether the lines are segments or infinite lines.
    :return: The intersection between the two lines, if it exists.
    """
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

    px = x1 + ua * (x2 - x1)
    py = y1 + ua * (y2 - y1)

    if segments and not (0 <= ua <= 1 and 0 <= ub <= 1):
        return None

    return px, py


def get_border_of_points(points: np.ndarray) -> tuple[int, int, int, int]:
    """Get the border of the points.

    :param points: The points.
    :return: The border of the points.
    """
    min_x = np.min(points[:, 0])
    min_y = np.min(points[:, 1])
    max_x = np.max(points[:, 0])
    max_y = np.max(points[:, 1])

    return int(min_x), int(min_y), int(max_x), int(max_y)
