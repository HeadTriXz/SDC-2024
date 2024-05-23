import cv2
import numpy as np

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from src.lane_assist.preprocessing.stitching import stitch_images
from src.utils.other import euclidean_distance
from src.utils.singleton_meta import SingletonMeta


class CalibrationData(metaclass=SingletonMeta):
    """A wrapper for a loaded camera calibrator.

    Attributes
    ----------
        input_shape: The shape of the input images (width, height).
        matrices: The perspective matrices for each camera.
        offsets: The offsets for each camera.
        output_shape: The output shape for the topdown image (width, height).
        pixels_per_meter: The amount of pixels per meter.
        ref_idx: The index of the reference camera.
        shapes: The shapes of the images for each camera (width, height).
        stitched_shape: The shape of the stitched image (width, height).
        topdown_matrix: The matrix for the topdown image.

    """

    input_shape: tuple[int, int]
    matrices: np.ndarray
    offsets: np.ndarray
    output_shape: tuple[int, int]
    pixels_per_meter: float = 0.0
    ref_idx: int
    shapes: np.ndarray
    stitched_shape: tuple[int, int]
    topdown_matrix: np.ndarray

    __pool: ThreadPoolExecutor

    def __init__(self) -> None:
        """Initialize the calibration data."""
        self.__pool = ThreadPoolExecutor()

    def transform(self, images: list[np.ndarray]) -> np.ndarray:
        """Transform the images to a topdown view.

        :param images: The images to transform.
        :return: The topdown image.
        """
        if self.topdown_matrix is None:
            raise ValueError("Calibrator has not been calibrated yet.")

        stitched = np.zeros(self.stitched_shape[::-1], dtype=np.uint8)

        futures = []
        for i, image in enumerate(images):
            if i == self.ref_idx:
                continue

            futures.append(self.__pool.submit(self._stitch_image, stitched, image, i))

        for future in futures:
            stitched = future.result()

        stitched = self._stitch_image(stitched, images[self.ref_idx], self.ref_idx)

        # Warp the stitched image to a topdown view
        return cv2.warpPerspective(
            stitched,
            self.topdown_matrix,
            self.output_shape,
            flags=cv2.INTER_NEAREST
        )

    def _stitch_image(self, stitched: np.ndarray, image: np.ndarray, idx: int) -> np.ndarray:
        """Stitch an image to the stitched image.

        :param stitched: The stitched image.
        :param image: The image to stitch.
        :param idx: The index of the camera.
        :return: The stitched image.
        """
        if image.shape[:2] != self.input_shape[::-1]:
            image = cv2.resize(image, self.shapes[idx], interpolation=cv2.INTER_NEAREST)

        warped = self._warp_image(image, idx)
        return stitch_images(stitched, warped, self.offsets[idx])

    def transform_point(self, x: int, y: int, shape: tuple[int, int]) -> tuple[int, int]:
        """Transform a point to a topdown view.

        :param x: The x-coordinate of the point.
        :param y: The y-coordinate of the point.
        :param shape: The shape of the image (width, height).
        :return: The transformed point.
        """
        x *= self.input_shape[0] / shape[0]
        y *= self.input_shape[1] / shape[1]

        x += self.offsets[self.ref_idx][0]
        y += self.offsets[self.ref_idx][1]

        src_point = np.array([[[x, y]]], dtype=np.float32)
        dst_point = cv2.perspectiveTransform(src_point, self.topdown_matrix)

        return int(dst_point[0][0][0]), int(dst_point[0][0][1])

    def get_distance(self, pixels: int) -> float:
        """Get the distance in meters from pixels.

        :param pixels: The amount of pixels.
        :return: The distance in meters.
        """
        return pixels / self.pixels_per_meter

    def get_distance_to_point(self, x: int, y: int, shape: tuple[int, int]) -> float:
        """Get the distance in meters from a coordinate.

        :param x: The x-coordinate in an image.
        :param y: The y-coordinate in an image.
        :param shape: The shape of the image (width, height).
        :return: The distance in meters.
        """
        x, y = self.transform_point(x, y, shape)
        return self.get_distance_to_transformed_point(x, y)

    def get_distance_to_transformed_point(self, x: int, y: int) -> float:
        """Get the distance in meters from a transformed point.

        :param x: The x-coordinate in the topdown image.
        :param y: The y-coordinate in the topdown image.
        :return: The distance in meters.
        """
        center = self.output_shape[0] // 2, self.output_shape[1]
        dist = euclidean_distance((x, y), center)

        return self.get_distance(int(dist))

    def get_distance_to_y(self, x: int, y: int, shape: tuple[int, int]) -> float:
        """Get the distance in meters from a coordinate, not considering the x-coordinate.

        :param x: The x-coordinate in an image.
        :param y: The y-coordinate in an image.
        :param shape: The shape of the image (width, height).
        :return: The distance in meters.
        """
        x, y = self.transform_point(x, y, shape)

        dist = int(self.output_shape[1] - y)
        return self.get_distance(dist)

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
        if idx == self.ref_idx:
            return image

        return cv2.warpPerspective(image, self.matrices[idx], self.shapes[idx], flags=cv2.INTER_NEAREST)

    @classmethod
    def load(cls, path: Path | str) -> "CalibrationData":
        """Load calibration data from a file.

        :param path: The path to the file.
        :return: The loaded calibration data.
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")

        data = np.load(path)

        calibration_data = cls()
        calibration_data.input_shape = tuple(data["input_shape"])
        calibration_data.matrices = data["matrices"]
        calibration_data.offsets = data["offsets"]
        calibration_data.output_shape = tuple(data["output_shape"])
        calibration_data.pixels_per_meter = float(data["pixels_per_meter"])
        calibration_data.ref_idx = int(data["ref_idx"])
        calibration_data.shapes = data["shapes"]
        calibration_data.stitched_shape = tuple(data["stitched_shape"])
        calibration_data.topdown_matrix = data["topdown_matrix"]

        return calibration_data
