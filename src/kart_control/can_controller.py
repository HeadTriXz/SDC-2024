import can
import struct
import threading

from common.constants import CAN_SEND_PERIOD, CANControlIdentifier, CANFeedbackIdentifier, Gear


def initialize_can_message(message_id: CANControlIdentifier) -> can.Message:
    """Initialize a CAN message.

    :param message_id: The identifier of the message.
    :return: The initialized CAN message.
    """
    return can.Message(arbitration_id=message_id, data=[0, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)


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
            message = self.bus.recv()
            if message.arbitration_id in self.__listeners:
                for listener in self.__listeners[message.arbitration_id]:
                    listener(message.data)
