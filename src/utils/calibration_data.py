import cv2
import numpy as np

from lane_assist.preprocessing.stitching import stitch_images
from lane_assist.preprocessing.utils.other import euclidean_distance
from pathlib import Path
from utils.singleton_meta import SingletonMeta


class CalibrationData(metaclass=SingletonMeta):
    """A wrapper for a loaded camera calibrator.

    Attributes
    ----------
        matrices: The perspective matrices for each camera.
        offsets: The offsets for each camera.
        output_shape: The output shape for the topdown image (width, height).
        pixels_per_meter: The amount of pixels per meter.
        ref_idx: The index of the reference camera.
        shapes: The shapes of the images for each camera (width, height).
        stitched_shape: The shape of the stitched image (width, height).
        topdown_matrix: The matrix for the topdown image.

    """

    matrices: np.ndarray
    offsets: np.ndarray
    output_shape: tuple[int, int]
    pixels_per_meter: float = 0.0
    ref_idx: int
    shapes: np.ndarray
    stitched_shape: tuple[int, int]
    topdown_matrix: np.ndarray

    def transform(self, images: list[np.ndarray]) -> np.ndarray:
        """Transform the images to a topdown view.

        :param images: The images to transform.
        :return: The topdown image.
        """
        if self.topdown_matrix is None:
            raise ValueError("Calibrator has not been calibrated yet.")

        # Stitch the warped images together
        stitched = np.zeros(self.stitched_shape[::-1], dtype=np.uint8)
        for i, img in enumerate(images):
            if i == self.ref_idx:
                continue

            warped = self._warp_image(img, i)
            stitched = stitch_images(stitched, warped, self.offsets[i])

        stitched = stitch_images(stitched, images[self.ref_idx], self.offsets[self.ref_idx])

        # Warp the stitched image to a topdown view
        return cv2.warpPerspective(
            stitched,
            self.topdown_matrix,
            self.output_shape,
            flags=cv2.INTER_NEAREST
        )

    def get_distance(self, pixels: int) -> float:
        """Get the distance in meters from pixels.

        :param pixels: The amount of pixels.
        :return: The distance in meters.
        """
        return pixels / self.pixels_per_meter

    def get_distance_to_point(self, x: int, y: int, topdown: bool) -> float:
        """Get the distance in meters from a coordinate.

        :param x: The x-coordinate in an image.
        :param y: The y-coordinate in an image.
        :param topdown: Whether the coordinates are in a topdown image.
        :return: The distance in meters.
        """
        if not topdown:
            offset = self.offsets[self.ref_idx]
            x += offset[0]
            y += offset[1]

            src_point = np.array([[[x, y]]], dtype=np.float32)
            dst_point = cv2.perspectiveTransform(src_point, self.topdown_matrix)

            x, y = dst_point[0][0]
            if x < 0 or y < 0:
                raise ValueError("Point is not on the ground plane.")

        center = self.output_shape[0] // 2, self.output_shape[1]
        dist = euclidean_distance((x, y), center)

        return self.get_distance(int(dist))

    def get_pixels(self, meters: float) -> int:
        """Get the amount of pixels from a distance in meters.

        :param meters: The distance in meters.
        :return: The amount of pixels.
        """
        return int(meters * self.pixels_per_meter)

    def _warp_image(self, image: np.ndarray, idx: int) -> np.ndarray:
        """Warp an image using a perspective matrix.

        :param image: The image to warp.
        :param idx: The camera to select the configuration for.
        :return: The warped image.
        """
        return cv2.warpPerspective(image, self.matrices[idx], self.shapes[idx], flags=cv2.INTER_NEAREST)

    @staticmethod
    def load(path: Path | str) -> "CalibrationData":
        """Load calibration data from a file.

        :param path: The path to the file.
        :return: The loaded calibration data.
        """
        data = np.load(Path(path))

        calibration_data = CalibrationData()
        calibration_data.matrices = data["matrices"]
        calibration_data.offsets = data["offsets"]
        calibration_data.output_shape = tuple(data["output_shape"])
        calibration_data.pixels_per_meter = data["pixels_per_meter"]
        calibration_data.ref_idx = data["ref_idx"]
        calibration_data.shapes = data["shapes"]
        calibration_data.stitched_shape = tuple(data["stitched_shape"])
        calibration_data.topdown_matrix = data["topdown_matrix"]

        return calibration_data
