import can
import logging
import struct
import threading
import time

from pathlib import Path

from src.constants import CAN_SEND_PERIOD, CANControlIdentifier, CANFeedbackIdentifier, Gear
from src.driving.can import ICANController, get_can_bus


def initialize_can_message(message_id: CANControlIdentifier) -> can.Message:
    """Initialize a CAN message.

    :param message_id: The identifier of the message.
    :return: The initialized CAN message.
    """
    return can.Message(arbitration_id=message_id, data=[0, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)


class CANController(ICANController):
    """A controller for the CAN bus.

    Attributes
    ----------
        bus (can.Bus): The CAN bus to use.

    """

    bus: can.Bus
    __listeners: dict[int, list[callable]]
    __thread: threading.Thread

    __recording_can: can.Bus = None
    __recorder: threading.Thread = None
    recording: bool = False

    def __init__(self, can_bus: can.Bus) -> None:
        """Initialize the CAN controller.

        :param can_bus: The CAN bus to use.
        """
        self.bus = can_bus
        self.bus.set_filters([{"can_id": CANFeedbackIdentifier.SPEED_SENSOR, "can_mask": 0xFFF, "extended": False}])

        self.__listeners = {}
        self.__thread = threading.Thread(target=self.__listen, daemon=True)

        self.__throttle_message = initialize_can_message(CANControlIdentifier.THROTTLE)
        self.__throttle_task = can_bus.send_periodic(self.__throttle_message, CAN_SEND_PERIOD)

        self.__brake_message = initialize_can_message(CANControlIdentifier.BRAKE)
        self.__brake_task = can_bus.send_periodic(self.__brake_message, CAN_SEND_PERIOD)

        self.__steering_message = initialize_can_message(CANControlIdentifier.STEERING)
        self.__steering_task = can_bus.send_periodic(self.__steering_message, CAN_SEND_PERIOD)

    def add_listener(self, message_id: CANFeedbackIdentifier, listener: callable) -> None:
        """Add a listener for a message.

        :param message_id: The identifier of the message.
        :param listener: The listener to add.
        """
        if message_id not in self.__listeners:
            self.__listeners[message_id] = []

        self.__listeners[message_id].append(listener)

    def set_brake(self, brake: int) -> None:
        """Set the percentage of the brake-force to apply.

        :param brake: The percentage of the brake-force to apply.
        """
        self.__brake_message.data = [brake, 0, 0, 0, 0, 0, 0, 0]
        self.__brake_task.modify_data(self.__brake_message)

    def set_steering(self, angle: float) -> None:
        """Set the angle of the steering wheel.

        :param angle: The angle of the steering wheel.
        """
        self.__steering_message.data = list(bytearray(struct.pack("f", angle))) + [0, 0, 195, 0]
        self.__steering_task.modify_data(self.__steering_message)

    def set_throttle(self, throttle: int, gear: Gear) -> None:
        """Set the percentage of the throttle to apply.

        :param throttle: The percentage of the throttle to apply.
        :param gear: The gear to put the go-kart in.
        """
        self.__throttle_message.data = [throttle, 0, gear, 0, 0, 0, 0, 0]
        self.__throttle_task.modify_data(self.__throttle_message)

    def start(self) -> None:
        """Start the CAN controller."""
        self.__thread.start()

    def __listen(self) -> None:
        """Listen to the CAN bus for messages."""
        while True:
            message = self.bus.recv(0.5)
            if message is not None and message.arbitration_id in self.__listeners:
                for listener in self.__listeners[message.arbitration_id]:
                    listener(message)

    def toggle_recording(self) -> None:
        """Toggle the recording of CAN messages.

        This function will start recording CAN messages if it is not already recording,
        and stop recording if it is already recording.

        On first call, it will create a new CAN bus to record messages.
        This is due to the virtual can not being able to receive its own messages.
        """
        if self.__recording_can is None:
            self.__recording_can = get_can_bus()
            self.__recording_can.set_filters(
                [{"can_id": can_id, "can_mask": 0xFFF, "extended": False} for can_id in CANControlIdentifier]
            )

        if not self.recording:
            path = Path(f"./data/can_recordings/can_{int(time.time())}.asc")
            path.parent.mkdir(parents=True, exist_ok=True)

            logging.info("Recording CAN messages to %s", path)
            self.__recorder = threading.Thread(target=self.__recording_thread, args=(path,), daemon=True)
            self.__recorder.start()

        else:
            self.recording = False
            self.__recorder.join(1)

            self.__recording_can.shutdown()
            self.__recording_can = None

            logging.info("Stopped recording CAN messages")

    def __recording_thread(self, filepath: Path) -> None:
        """Record CAN messages into a .asc file."""
        self.recording = True

        with can.ASCWriter(filepath) as writer:
            while self.recording:
                msg = self.__recording_can.recv(1)
                if msg is not None:
                    writer.on_message_received(msg)
