import struct

import can


class BasicDriving:
    def __init__(self, can_bus: can.Bus):
        self.can = can_bus

    ready = False

    def arm(self):
        """ release the brake and set the throttle to 0."""
        self.set_throttle(0, 0)
        self.set_brake(0)

    def set_throttle(self, throttle, direction):
        if not self.ready:
            return

        self.can.send(can.Message(arbitration_id=0x200, data=[throttle, 0, direction, 0, 0, 0, 0, 0]))

    def set_steering(self, angle):
        if not self.ready:
            return

        self.can.send(
            can.Message(arbitration_id=0x100, data=(list(bytearray(struct.pack("f", float(angle)))) + [0, 0, 195, 0])))

    def set_brake(self, brake):
        if not self.ready:
            return

        self.can.send(can.Message(arbitration_id=0x300, data=[brake, 0, 0, 0, 0, 0, 0, 0]))