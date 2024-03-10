import struct
import threading

import can

from common.constants import CANControlIdentifier, CANFeedbackIdentifier, Gear


class CANController:
    """A controller for the CAN bus.

    Attributes
    ----------
        bus (can.Bus): The CAN bus to use.

    """

    bus: can.Bus
    __listeners: dict[int, list[callable]]
    __thread: threading.Thread

    def __init__(self, can_bus: can.Bus) -> None:
        """Initialize the CAN controller.

        :param can_bus: The CAN bus to use.
        """
        self.bus = can_bus
        self.__listeners = {}
        self.__thread = threading.Thread(target=self.__listen, daemon=True)

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
        data = [brake, 0, 0, 0, 0, 0, 0, 0]
        message = can.Message(arbitration_id=CANControlIdentifier.BRAKE, data=data)

        self.bus.send(message)

    def set_steering(self, angle: int) -> None:
        """Set the angle of the steering wheel.

        :param angle: The angle of the steering wheel.
        """
        data = list(bytearray(struct.pack("f", float(angle)))) + [0, 0, 195, 0]
        message = can.Message(arbitration_id=CANControlIdentifier.STEERING, data=data)

        self.bus.send(message)

    def set_throttle(self, throttle: int, gear: Gear) -> None:
        """Set the percentage of the throttle to apply.

        :param throttle: The percentage of the throttle to apply.
        :param gear: The gear to put the go-kart in.
        """
        data = [throttle, 0, gear, 0, 0, 0, 0, 0]
        message = can.Message(arbitration_id=CANControlIdentifier.THROTTLE, data=data)

        self.bus.send(message)

    def start(self) -> None:
        """Start the CAN controller."""
        self.__thread.start()

    def __listen(self) -> None:
        """Listen to the CAN bus for messages."""
        while True:
            message = self.bus.recv()
            if message.arbitration_id in self.__listeners:
                for listener in self.__listeners[message.arbitration_id]:
                    listener(message.data)
