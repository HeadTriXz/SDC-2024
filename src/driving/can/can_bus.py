import can
import logging
import os


def get_can_bus(channel: str = "can0", bitrate: int = 500000) -> can.ThreadSafeBus:
    """Create a CAN bus using the specified channel and bitrate.

    If the interface creation fails, a virtual interface is created instead.

    :param channel: The channel to use.
    :param bitrate: The bitrate to use.
    :return: The created CAN bus.
    """
    status = os.system("ip link set can0 type can bitrate 500000")
    if status == 0:
        os.system("ip link set can0 up")
        return can.ThreadSafeBus(interface="socketcan", channel=channel, bitrate=bitrate)

    logging.warning("Failed to create CAN interface, using virtual interface instead.")
    return can.ThreadSafeBus(interface="virtual", channel="vcan0")
