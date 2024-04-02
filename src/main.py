import logging
import threading
import time
from config import config
from telemetry.webapp.telemetry_server import TelemetryServer
from utils.video_stream import VideoStream


def send_images(cam_left: VideoStream, telem_server: TelemetryServer) -> None:
    """Send images/text to the telemetry server."""
    counter = 0
    while cam_left.has_next():
        telem_server.websocket_handler.send_text("counter", f"Frame: {counter}")
        counter += 1
        time.sleep(0.1)


def main() -> None:
    """Start the main loop."""
    # Load cameras
    cam_left = VideoStream(config.camera_ids.left)
    cam_left.start()

    # load the telemetry server
    server = TelemetryServer()
    thread = threading.Thread(target=send_images, args=(cam_left, server), daemon=True)

    server.start()
    thread.start()
    input("Press Enter to exit...")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
