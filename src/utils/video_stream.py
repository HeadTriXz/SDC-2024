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
        frame_rate (CameraFramerate): The frame rate of the video stream.
        resolution (CameraResolution): The resolution of the video stream.

    """

    id: int
    capture: cv2.VideoCapture
    frame_rate: CameraFramerate
    resolution: CameraResolution

    __initialized: bool = False
    __instances: dict[int, "VideoStream"] = {}

    __frame: np.ndarray
    __ret: bool
    __stopped: bool = True
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
        self.resolution = resolution
        self.frame_rate = frame_rate

    @property
    def stopped(self) -> bool:
        """Whether the video stream is stopped."""
        return self.__stopped

    def has_next(self) -> bool:
        """Checks if the video stream has more frames."""
        return self.__ret

    def next(self) -> np.ndarray:
        """Reads the next frame from the video stream."""
        return self.__frame

    def start(self) -> None:
        """Starts the video stream."""
        if not self.__stopped:
            return

        if not self.__initialized:
            raise ValueError("Cannot restart a detached video stream.")

        self.__stopped = False
        self.__init_capture()

        self.__thread = Thread(target=self.update, daemon=True)
        self.__thread.start()

    def stop(self, detach: bool = False) -> None:
        """Stops the video stream.

        :param detach: Whether to detach the stream.
        """
        if detach:
            self.__initialized = False
            self.__instances.pop(self.id, None)

        if self.__stopped:
            return

        self.__stopped = True
        self.__thread.join()
        self.capture.release()

    def update(self) -> None:
        """Reads frames from the video stream."""
        while True:
            if self.__stopped:
                break

            self.__ret, self.__frame = self.capture.read()
            if not self.__ret:
                self.stop()
                break

    def __init_capture(self) -> None:
        """Initializes the video capture object."""
        self.capture = cv2.VideoCapture(self.id, get_camera_backend())
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.capture.set(cv2.CAP_PROP_FOCUS, 0)
        self.capture.set(cv2.CAP_PROP_FPS, self.frame_rate)

        # Initialize the video stream.
        self.__ret, self.__frame = self.capture.read()
        if not self.__ret:
            raise ValueError(f"Failed to open camera {self.id}.")
