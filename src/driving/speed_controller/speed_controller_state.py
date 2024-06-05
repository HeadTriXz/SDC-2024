from enum import IntEnum


class SpeedControllerState(IntEnum):
    """The state of the speed controller."""

    STOPPED = 0
    WAITING_TO_STOP = 1
    DRIVING = 2
    PARKING = 3
