import os

import can


def get_can_interface(channel: str = "can0", bitrate: int = 500000) -> can.ThreadSafeBus:
    """Create a CAN interface using the specified channel and bitrate.

    If the interface creation fails, a virtual interface is created instead.

    :param channel: Channel to use.
    :param bitrate: Bitrate to use.
    :return: A CAN interface.
    """
    status = os.system("ip link set can0 type can bitrate 500000")
    if status == 0:
        os.system("ip link set can0 up")
        return can.ThreadSafeBus(interface="socketcan", channel=channel, bitrate=bitrate)
    return can.ThreadSafeBus(interface="virtual", channel="vcan0")
