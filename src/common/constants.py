from enum import IntEnum


class CANControlIdentifier(IntEnum):
    """
    The identifiers for the CAN messages sent to the go-kart.
    """

    BRAKE    = 0x110
    STEERING = 0x220
    THROTTLE = 0x330

class CANFeedbackIdentifier(IntEnum):
    """
    The identifiers for the CAN messages received from the go-kart.
    """

    BRAKE    = 0x126
    SPEED    = 0x15e
    STEERING = 0x12C
    THROTTLE = 0x120

class Gear(IntEnum):
    """
    The gear of the go-kart.
    """

    NEUTRAL = 0
    DRIVE   = 1
    REVERSE = 2

class SpeedMode(IntEnum):
    """
    The mode of the speed controller.
    """

    SLOW =      0
    MEDIUM =    1
    FAST =      2
    VERY_FAST = 3
