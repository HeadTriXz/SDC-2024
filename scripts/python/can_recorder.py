import can
import logging
import threading
import time

from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from src.constants import CANControlIdentifier
from src.driving.can import CANController, get_can_bus
from src.driving.gamepad import EventType, Gamepad, GamepadButton
from src.driving.modes import ManualDriving


class CANRecorder:
    """Procedure to record CAN message.

    This procedure can be used to record messages send on the CAN bus.
    This will record all the messages we can send and not the ones we receive.
    """

    __can_bus: can.Bus
    __recording: bool = False
    __recorder: threading.Thread = None

    def __init__(self) -> None:
        """Procedure to record CAN messages."""
        self.__can_bus = get_can_bus()
        self.__can_bus.set_filters(
            [{"can_id": can_id, "can_mask": 0xFFF, "extended": False} for can_id in CANControlIdentifier]
        )

    def toggle_recording(self) -> None:
        """Toggle the recording of CAN messages.

        This function will start recording CAN messages if it is not already recording,
        and stop recording if it is already recording.
        """
        if not self.__recording:
            path = Path(f"./data/can_recordings/{datetime.now().strftime("%m_%d_%Y_%H_%M_%S")}.asc")
            path.parent.mkdir(parents=True, exist_ok=True)

            logging.info("Recording CAN messages to %s", path)
            self.__recorder = threading.Thread(target=self.__recording_thread, args=(path,), daemon=True)
            self.__recorder.start()
        else:
            self.__recording = False
            self.__recorder.join(1)

            self.__can_bus.shutdown()
            self.__can_bus = None

            logging.info("Stopped recording CAN messages")

    def __recording_thread(self, filepath: Path) -> None:
        """Record CAN messages into a .asc file."""
        self.__recording = True

        with can.ASCWriter(filepath) as writer:
            while self.__recording:
                msg = self.__can_bus.recv(1)
                if msg is not None:
                    writer.on_message_received(msg)


def create_toggle_callback(can_recorder: CANRecorder, gamepad: Gamepad) -> Callable[[Any, Any], None]:
    """Create the callback for the toggle of the CAN recording."""

    def __toggle(*_args: Any, **_kwargs: Any) -> None:
        can_recorder.toggle_recording()
        gamepad.vibrate()

    return __toggle


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    can_bus = get_can_bus()
    can_controller = CANController(can_bus)
    can_controller.start()

    gamepad = Gamepad()
    gamepad.start()

    controller_driving = ManualDriving(gamepad, can_controller)
    controller_driving.start()

    can_recorder = CANRecorder()
    gamepad.add_listener(GamepadButton.LB, EventType.LONG_PRESS, create_toggle_callback(can_recorder, gamepad))

    while True:
        time.sleep(1)
