from typing import Optional

import cv2
import numpy as np

from lane_assist.preprocessing.utils.corners import get_transformed_corners


def segment_intersection(seg1, seg2):
    """
    Helper function to check if two line segments intersect.
    seg1 and seg2 are numpy arrays of shape (2, 2), representing the two line segments.
    Returns the intersection point if there is one, otherwise returns None.
    """
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    A, B = seg1
    C, D = seg2

    if intersect(A, B, C, D):
        # Calculate intersection point
        dx1 = B[0] - A[0]
        dy1 = B[1] - A[1]
        dx2 = D[0] - C[0]
        dy2 = D[1] - C[1]
        denominator = dx1 * dy2 - dy1 * dx2

        if denominator != 0:
            t1 = ((C[0] - A[0]) * dy2 - (C[1] - A[1]) * dx2) / denominator
            intersection = np.array([A[0] + t1 * dx1, A[1] + t1 * dy1])
            return intersection
    return None


def find_self_intersection(polygon):
    """
    Function to find self-intersection points in a polygon defined by a numpy array of points.
    Returns the intersection point if there is one, otherwise returns None.
    """
    n = len(polygon)

    # Iterate through all pairs of edges
    for i in range(n):
        seg1 = np.array([polygon[i], polygon[(i + 1) % n]])

        for j in range(i + 2, n):
            seg2 = np.array([polygon[j], polygon[(j + 1) % n]])

            # Skip adjacent edges and non-intersecting edges
            if i == 0 and j == n - 1:
                continue

            intersection = segment_intersection(seg1, seg2)
            if intersection is not None:
                return intersection

    return None


def warp_image(image: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Warp an image using a perspective matrix.

    :param image: The image to warp.
    :param matrix: The perspective matrix.
    :return: The warped image and the amount cropped from the top.
    """
    min_x, min_y, max_x, max_y = get_transformed_corners(matrix, (image.shape[0], image.shape[1]))

    # DEBUGGING
    # h, w = image.shape[:2]
    # src_points = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], dtype=np.float32)
    # dst_points = cv2.perspectiveTransform(src_points, matrix).astype(int).reshape(-1, 2)
    #
    # intersections = find_self_intersection(dst_points)
    # if intersections is not None:
    #     max_y = min(max_y, intersections[1])
    #
    #     min_x = np.inf
    #     max_x = -np.inf
    #
    #     for point in dst_points:
    #         if point[1] < max_y:
    #             min_x = min(min_x, point[0])
    #             max_x = max(max_x, point[0])
    # END DEBUGGING

    width = int(max_x - min_x)
    height = int(max_y - min_y)

    adjusted_matrix = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]])
    adjusted_matrix = np.dot(adjusted_matrix, matrix)

    return cv2.warpPerspective(image, adjusted_matrix, (width, height), flags=cv2.INTER_LINEAR)
