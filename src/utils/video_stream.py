import cv2
import numpy as np
import sys

from threading import Thread

from src.constants import CameraFramerate, CameraResolution


def get_camera_backend() -> int:
    """Get the camera backend based on the platform.

    :return: The camera backend.
    """
    match sys.platform:
        case "linux":
            return cv2.CAP_V4L2
        case "win32":
            return cv2.CAP_DSHOW
        case "darwin":
            return cv2.CAP_AVFOUNDATION
        case _:
            raise ValueError("Unsupported platform.")


class VideoStream:
    """A class to read frames from a video stream.

    Attributes
    ----------
        id (int): The camera ID.
        capture (cv2.VideoCapture): The OpenCV video capture object.
        stopped (bool): A flag to indicate if the video stream is stopped.

    """

    id: int
    capture: cv2.VideoCapture
    stopped: bool = True

    __initialized: bool = False
    __instances: dict[int, "VideoStream"] = {}

    __frame: np.ndarray
    __ret: bool
    __thread: Thread

    def __new__(
        cls,
        camera_id: int,
        resolution: CameraResolution = CameraResolution.HD,  # noqa: ARG003
        frame_rate: CameraFramerate = CameraFramerate.FPS_60,  # noqa: ARG003
    ) -> "VideoStream":
        """Create a new instance of the video stream.

        :param camera_id: The camera ID.
        :param resolution: The resolution of the video stream (default is 720p).
        :param frame_rate: The frame rate of the video stream (default is 60 FPS).
        :return: The video stream instance.
        """
        if camera_id not in cls.__instances:
            cls.__instances[camera_id] = super().__new__(cls)
            cls.__instances[camera_id].__initialized = False

        return cls.__instances[camera_id]

    def __init__(
        self,
        camera_id: int,
        resolution: CameraResolution = CameraResolution.HD,
        frame_rate: CameraFramerate = CameraFramerate.FPS_60,
    ) -> None:
        """Initializes the video stream.

        :param camera_id: The camera ID.
        :param resolution: The resolution of the video stream (default is 720p).
        :param frame_rate: The frame rate of the video stream (default is 60 FPS).
        """
        if self.__initialized:
            return

        self.__initialized = True

        self.id = camera_id
        self.capture = cv2.VideoCapture(camera_id, get_camera_backend())
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.capture.set(cv2.CAP_PROP_FPS, frame_rate)

        # Initialize the video stream.
        self.__ret, self.__frame = self.capture.read()
        if not self.__ret:
            raise ValueError(f"Failed to open camera {self.id}.")

        # Start the thread to read frames from the video stream.
        self.__thread = Thread(target=self.update, daemon=True)

    def start(self) -> None:
        """Starts the video stream."""
        self.stopped = False

        if not self.__thread.is_alive():
            self.__thread.start()

    def update(self) -> None:
        """Reads frames from the video stream."""
        while True:
            if self.stopped:
                break

            self.__ret, self.__frame = self.capture.read()
            if not self.__ret:
                self.stop()
                break

    def has_next(self) -> bool:
        """Checks if the video stream has more frames."""
        return self.__ret

    def next(self) -> np.ndarray:
        """Reads the next frame from the video stream."""
        return self.__frame

    def stop(self) -> None:
        """Stops the video stream."""
        self.stopped = True
        self.capture.release()
        self.__thread.join()
