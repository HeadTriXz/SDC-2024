from enum import Enum, IntEnum


class CANControlIdentifier(IntEnum):
    """The identifiers for the CAN messages sent to the go-kart."""

    BRAKE = 0x110
    STEERING = 0x220
    THROTTLE = 0x330


class CANFeedbackIdentifier(IntEnum):
    """The identifiers for the CAN messages received from the go-kart."""

    BRAKE = 0x710
    SPEED_SENSOR = 0x440
    STEERING_ECU = 0x720
    STEERING_SENSOR = 0x1E5
    THROTTLE = 0x730


class CameraResolution(tuple[int, int], Enum):
    """The camera resolutions that the Logitech StreamCam supports."""

    FHD = (1920, 1080)
    HD = (1280, 720)
    VGA = (848, 480)


class CameraFramerate(float, Enum):
    """The camera framerates that the Logitech StreamCam supports."""

    FPS_60 = 60
    FPS_30 = 30
    FPS_24 = 24
    FPS_20 = 20
    FPS_15 = 15
    FPS_10 = 10


class Gear(IntEnum):
    """The gear of the go-kart."""

    NEUTRAL = 0
    DRIVE = 1
    REVERSE = 2


class SpeedMode(IntEnum):
    """The mode of the speed controller."""

    SLOW = 0
    MEDIUM = 1
    FAST = 2
    VERY_FAST = 3
