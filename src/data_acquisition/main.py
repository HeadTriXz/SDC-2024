import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from queue import Queue
from typing import Optional

import can
import cv2

CAN_MSG_SENDING_SPEED = .040  # 25Hz

class CameraResolution:
    """
    The camera resolutions that the Logitech StreamCam supports.
    """

    FHD = (1920, 1080)
    HD = (1280, 720)
    VGA = (848, 480)

class CameraFramerate:
    """
    The camera framerates that the Logitech StreamCam supports.
    """

    FPS_60 = 60
    FPS_30 = 30
    FPS_24 = 24
    FPS_20 = 20
    FPS_15 = 15
    FPS_10 = 10

class CanListener:
    """
    A can listener that listens for specific messages and stores their latest values.
    """

    _id_conversion = {
        0x12c: 'steering',
        0x120: 'throttle',
        0x126: 'brake',
        0x15e: 'speed_sensor'
    }

    def __init__(self, bus: can.Bus):
        self.bus = bus
        self.thread = threading.Thread(target=self._listen, args=(), daemon=True)
        self.running = False
        self.data = {
            'steering': None,
            'throttle': None,
            'brake': None,
            'speed_sensor': None
        }

    def start_listening(self):
        if self.bus is None:
            return

        self.running = True
        self.thread.start()

    def stop_listening(self):
        self.running = False

    def get_new_values(self):
        values = self.data
        return values

    def _listen(self):
        while self.running:
            message: Optional[can.Message] = self.bus.recv(.5)
            if message and message.arbitration_id in self._id_conversion:
                self.data[self._id_conversion[message.arbitration_id]] = message.data


class ImageWorker:
    """
    A worker that writes images to disk.
    """

    def __init__(self, image_queue: Queue, folder_name):
        self.queue = image_queue
        self.thread = threading.Thread(target=self._process, args=(), daemon=True)
        self.folder_name = folder_name

    def start(self):
        self.thread.start()

    def stop(self):
        self.queue.join()

    def put(self, data):
        self.queue.put(data)

    def _process(self):
        while True:
            timestamp, image = self.queue.get()
            cv2.imwrite(self.folder_name + f'/{timestamp}.jpg', image)
            self.queue.task_done()


class CanWorker:
    """
    A worker that writes can-message values to disk.
    """

    def __init__(self, can_queue: Queue, folder_name):
        self.queue = can_queue
        self.thread = threading.Thread(target=self._process, args=(), daemon=True)
        self.folder_name = folder_name
        self.file_pointer = open('data/' + self.folder_name + '.csv', 'w')
        print('Steering|Throttle|Brake|Speed|Image', file=self.file_pointer)

    def start(self):
        self.thread.start()

    def stop(self):
        self.queue.join()
        self.file_pointer.close()

    def put(self, data):
        self.queue.put(data)

    def _process(self):
        while True:
            timestamp, values = self.queue.get()
            print(
                f'\"{",".join([str(x) for x in values["steering"] or []])}\"|\"{",".join([str(x) for x in values["throttle"] or []])}\"|\"{",".join([str(x) for x in values["brake"] or []])}\"|\"{",".join([str(x) for x in values["speed_sensor"] or []])}\"|\"' + self.folder_name + f'/{timestamp}.jpg\"',
                file=self.file_pointer)
            self.queue.task_done()


def main():
    folder_name = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    folder = Path("images/" + folder_name)

    print('Initializing...', file=sys.stderr)
    if not folder.exists():
        folder.mkdir(parents=True, exist_ok=True)

    bus = initialize_can()
    cam1, cam2, cam3 = initialize_cameras()
    cam1_folder = folder / 'cam1'
    cam2_folder = folder / 'cam2'
    cam3_folder = folder / 'cam3'

    if not cam1_folder.exists():
        cam1_folder.mkdir(parents=True, exist_ok=True)
    if not cam2_folder.exists():
        cam2_folder.mkdir(parents=True, exist_ok=True)
    if not cam3_folder.exists():
        cam3_folder.mkdir(parents=True, exist_ok=True)

    cam1_worker = ImageWorker(Queue(), str(cam1_folder))
    cam2_worker = ImageWorker(Queue(), str(cam2_folder))
    cam3_worker = ImageWorker(Queue(), str(cam3_folder))

    cam1_worker.start()
    cam2_worker.start()
    cam3_worker.start()

    can_listener = CanListener(bus)
    can_listener.start_listening()
    can_worker = CanWorker(Queue(), folder_name)
    can_worker.start()

    print('Recording...', file=sys.stderr)
    try:
        while True:
            timestamp = time.time()

            if cam1:
                _, frame = cam1.read()
                cam1_worker.put((timestamp, frame))

            if cam2:
                _, frame = cam2.read()
                cam2_worker.put((timestamp, frame))

            if cam3:
                _, frame = cam3.read()
                cam3_worker.put((timestamp, frame))

            values = can_listener.get_new_values()
            can_worker.put((timestamp, values))
    except KeyboardInterrupt:
        pass

    print('Stopping...', file=sys.stderr)
    can_listener.stop_listening()
    can_worker.stop()

    if cam1:
        cam1.release()
    if cam2:
        cam2.release()
    if cam3:
        cam3.release()

    cam1_worker.stop()
    cam2_worker.stop()
    cam3_worker.stop()


def initialize_can() -> can.Bus | None:
    """
    Set up the can bus interface and apply filters for the messages we're interested in.
    """

    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
        bus.set_filters([
            {'can_id': 0x12c, 'can_mask': 0xfff, 'extended': True},  # Steering
            {'can_id': 0x120, 'can_mask': 0xfff, 'extended': True},  # Throttle
            {'can_id': 0x126, 'can_mask': 0xfff, 'extended': True},  # Brake
            {'can_id': 0x15e, 'can_mask': 0xfff, 'extended': True},  # Speed sensor
        ])
        return bus
    except Exception as e:
        print(f'Error initializing CAN: {e}', file=sys.stderr)
        return None

def initialize_camera(device: int | str) -> cv2.VideoCapture | None:
    capture = cv2.VideoCapture(device)
    if capture is None or not capture.isOpened():
        return None

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CameraResolution.FHD[0])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CameraResolution.FHD[1])
    capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    capture.set(cv2.CAP_PROP_FOCUS, 0)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
    capture.set(cv2.CAP_PROP_FPS, CameraFramerate.FPS_20)

    return capture


def initialize_cameras() -> tuple[cv2.VideoCapture, cv2.VideoCapture, cv2.VideoCapture]:
    """
    Initialize the opencv camera capture device.
    """

    return initialize_camera(0), initialize_camera(2), initialize_camera(4)


if __name__ == '__main__':
    main()
