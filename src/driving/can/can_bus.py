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
    if os.system(f"ip link show {channel}") == 0:
        os.system(f"ip link set {channel} type can bitrate {bitrate}")
        os.system(f"ip link set {channel} up")

        return can.ThreadSafeBus(interface="socketcan", channel=channel, bitrate=bitrate)

    logging.warning("Failed to create CAN interface, using virtual interface instead.")
    return can.ThreadSafeBus(interface="virtual", channel="vcan0")
