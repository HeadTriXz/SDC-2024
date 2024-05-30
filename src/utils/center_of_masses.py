import numba
import numpy as np


@numba.njit
def label_components(image: np.ndarray) -> tuple[np.ndarray, int]:
    """Label the connected components in the image.

    :param image: The image to label.
    :return: The labeled image and the number of labels.
    """
    rows, cols = image.shape
    labels = np.zeros_like(image, dtype=np.int32)
    num_labels = 0

    def flood_fill(row: int, col: int, label: int) -> None:
        stack = [(row, col)]
        while stack:
            x, y = stack.pop()
            if labels[x, y] == 0 and image[x, y] > 0:
                labels[x, y] = label
                if x > 0:
                    stack.append((x - 1, y))
                if x < rows - 1:
                    stack.append((x + 1, y))
                if y > 0:
                    stack.append((x, y - 1))
                if y < cols - 1:
                    stack.append((x, y + 1))

    for r in range(rows):
        for c in range(cols):
            if image[r, c] > 0 and labels[r, c] == 0:
                num_labels += 1
                flood_fill(r, c, num_labels)

    return labels, num_labels


@numba.njit
def compute_centroids(labels: np.ndarray, num_labels: int) -> tuple[np.ndarray, np.ndarray]:
    """Compute the centroids of the labels.

    :param labels: The labels.
    :param num_labels: The amount of labels.
    :return: The centroids and counts of the labels.
    """
    centroids = np.zeros((num_labels, 2), dtype=np.float64)
    counts = np.zeros(num_labels, dtype=np.int32)

    rows, cols = labels.shape
    for r in range(rows):
        for c in range(cols):
            label = labels[r, c]
            if label > 0:
                centroids[label - 1, 0] += r
                centroids[label - 1, 1] += c
                counts[label - 1] += 1

    for i in range(num_labels):
        if counts[i] > 0:
            centroids[i] /= counts[i]

    return centroids, counts


@numba.njit
def center_of_masses(image: np.ndarray, target: int, min_pixels: int = 1) -> tuple[int, int] | None:
    """Get the center of the nearest cluster of pixels.

    :param image: The image to process.
    :param target: The x-coordinate to target (nearest to us).
    :param min_pixels: The minimum number of pixels in the cluster.
    :return: The center of the nearest cluster of pixels (x, y).
    """
    labels, num_features = label_components(image)
    if num_features == 0:
        return None

    centroids, counts = compute_centroids(labels, num_features)

    nearest = None
    nearest_dist = np.inf

    for i in range(num_features):
        if counts[i] >= min_pixels:
            center = centroids[i]
            distance = abs(target - center[1])

            if distance < nearest_dist:
                nearest = center
                nearest_dist = distance

    if nearest is None:
        return None

    return int(nearest[1]), int(nearest[0])
