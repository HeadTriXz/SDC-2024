from threading import Thread
from pathlib import Path
from object_recognition.object_controller import ObjectController
from utils.video_stream import VideoStream
from ultralytics import YOLO


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
            results = self.model.track(frame, imgsz=1280, conf=0.6, persist=True, device="cpu")

            self.controller.handle(results[0].boxes)

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
        ov_path = path.parent / f"{path.stem}_openvino_model/"
        if not ov_path.exists():
            model = YOLO(path)
            model.export(format="openvino")

        model = YOLO(path.parent / f"{path.stem}_openvino_model/")
        return ObjectDetector(model, controller, camera_id)


if __name__ == "__main__":
    from driving.can_controller import CANController
    from driving.speed_controller import SpeedController
    from object_recognition.handlers.pedestrian_handler import PedestrianHandler
    from object_recognition.handlers.speed_limit_handler import SpeedLimitHandler
    import can

    can_bus = can.Bus(interface="virtual", channel="can0", bitrate=500000)
    can_controller = CANController(can_bus)
    speed_controller = SpeedController(can_controller)

    controller = ObjectController(speed_controller)
    controller.add_handler(PedestrianHandler(controller))
    controller.add_handler(SpeedLimitHandler(controller))

    detector = ObjectDetector.from_model("../../resources/models/best.pt", controller, 0)
    can_controller.start()
    speed_controller.start()
    detector.start()

    while True:
        pass
