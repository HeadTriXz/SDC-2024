import time
from config import config
from object_recognition.object_controller import ObjectController
from pathlib import Path
from threading import Thread
from ultralytics import YOLO
from utils.video_stream import VideoStream


class ObjectDetector:
    """A class to detect objects in a video stream.

    Attributes
    ----------
        controller (ObjectController): The object controller.
        model (YOLO): The object detection model.
        stream (VideoStream): The video stream.

    """

    controller: ObjectController
    model: YOLO
    stream: VideoStream
    __thread: Thread

    def __init__(self, model: YOLO, controller: ObjectController, camera_id: int) -> None:
        """Initializes the object detector.

        :param model: The object detection model.
        :param controller: The object controller.
        :param camera_id: The camera ID.
        """
        self.controller = controller
        self.model = model
        self.stream = VideoStream(camera_id)
        self.__thread = Thread(target=self.__track_video_stream, daemon=True)

    def start(self) -> None:
        """Start looking for objects in the video stream."""
        self.stream.start()
        self.__thread.start()

    def __track_video_stream(self) -> None:
        """Track the objects in the video stream."""
        while self.stream.has_next():
            frame = self.stream.next()
            results = self.model.track(
                frame,
                imgsz=config.object_detection.image_size,
                conf=config.object_detection.min_confidence,
                verbose=config.object_detection.verbose,
                persist=True,
                device="cpu",
            )

            self.controller.handle(results[0].boxes)
            time.sleep(0)

        self.stream.stop()

    @staticmethod
    def from_model(path: str | Path, controller: ObjectController, camera_id: int) -> "ObjectDetector":
        """Creates a new instance of the object detector from a model file.

        :param path: The path to the model file.
        :param controller: The object controller.
        :param camera_id: The camera ID.
        :return: The object detector instance.
        """
        path = Path(path)
        ov_path = path.parent / f"{path.stem}_int8_openvino_model/"
        if not ov_path.exists():
            model = YOLO(path)
            model.export(format="openvino", int8=True)

        model = YOLO(ov_path)
        return ObjectDetector(model, controller, camera_id)
