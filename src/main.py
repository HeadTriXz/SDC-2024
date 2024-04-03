import logging
import threading
import time
from config import config
from telemetry.webapp.telemetry_server import TelemetryServer
from utils.video_stream import VideoStream
import sys
from telemetry.webapp.logger import Loggneer
from subprocess import Popen, PIPE



def send_images(cam_left: VideoStream, telem_server: TelemetryServer) -> None:
    """Send images/text to the telemetry server."""
    counter = 0
    while cam_left.has_next():
        telem_server.websocket_handler.send_text("counter", f"Frame: {counter}")
        counter += 1
        time.sleep(0.1)

def pyton_turorial():
    count = 0
    while True:
        logging.info(f"Count: {count}")
        count += 1
        time.sleep(0.1)

def main() -> None:
    """Start the main loop."""
    cam_left = VideoStream(config.camera_ids.left)
    cam_left.start()
    server = TelemetryServer()
    logger = Loggneer(server)
    sys.stdout = logger

    logging.basicConfig(level=logging.INFO, stream=logger)

    thread = threading.Thread(target=send_images, args=(cam_left, server), daemon=True)
    thread.start()
    siepels = threading.Thread(target=pyton_turorial, daemon=True)
    siepels.start()
    server.start()
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()
