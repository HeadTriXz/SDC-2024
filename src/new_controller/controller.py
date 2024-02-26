from enum import Enum, IntEnum
from threading import Timer

from inputs import get_gamepad


class ControllerButton(IntEnum):
    A = 0
    B = 1
    X = 2
    Y = 3
    DPAD_UP = 4
    DPAD_DOWN = 5
    DPAD_LEFT = 6
    DPAD_RIGHT = 7
    START = 8
    SELECT = 9
    LB = 10
    RB = 11
    LS = 12  # Left Stick
    RS = 13  # Right Stick
    # Add more buttons as needed


class ControllerAxis(IntEnum):
    LS_X = 0  # Left Stick X
    LS_Y = 1  # Left Stick Y
    RS_X = 2  # Right Stick X
    RS_Y = 3  # Right Stick Y
    LT = 4  # Left Trigger
    RT = 5  # Right Trigger


class EventType(Enum):
    BUTTON_DOWN = "button_down"
    BUTTON_UP = "button_up"
    LONG_PRESS = "long_press"
    SHORT_PRESS = "short_press"
    AXIS_CHANGED = "axis_changed"


class Controller:
    def __init__(self):
        self._buttons = {}
        self._axes = {}
        self._listeners = {}
        self._last_buttons = {}
        self._long_press_timers = {}

    def start(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == "Key":
                    self._handle_button_event(event)
                elif event.ev_type == "Axis":
                    self._handle_axis_event(event)

    def add_listener(self, event_type, button_or_axis, callback):
        if event_type not in EventType:
            raise ValueError(f"Invalid event type: {event_type}")
        if button_or_axis not in (ControllerButton, ControllerAxis):
            raise ValueError(f"Invalid button or axis: {button_or_axis}")
        key = (event_type, button_or_axis)
        if key not in self._listeners:
            self._listeners[key] = []
        self._listeners[key].append(callback)

    def _handle_button_event(self, event):
        button = ControllerButton(event.code)
        if event.state:
            self._buttons[button] = True
            self._check_events(EventType.BUTTON_DOWN, button)
            if button not in self._last_buttons:
                self._start_long_press_timer(button)
        else:
            self._buttons[button] = False
            self._check_events(EventType.BUTTON_UP, button)
            self._cancel_long_press_timer(button)
        self._last_buttons[button] = event.state

    def _handle_axis_event(self, event):
        axis = ControllerAxis(event.code)
        # Normalize to -1 to 1 for joysticks, 0 to 1 for triggers
        if axis in (
            ControllerAxis.LS_X,
            ControllerAxis.LS_Y,
            ControllerAxis.RS_X,
            ControllerAxis.RS_Y,
        ):
            value = (event.state - 1) / 2  # Normalize to -1 to 1 for joysticks
        else:
            value = (event.state + 1) / 2  # Normalize to 0 to 1 for triggers
        self._axes[axis] = value
        self._check_events(EventType.AXIS_CHANGED, (axis, value))

    def _check_events(self, event_type, data):
        key = (event_type, data)
        if key in self._listeners:
            for callback in self._listeners[key]:
                callback(event_type, data)

    def _start_long_press_timer(self, button, timeout=0.5):
        def timer_callback():
            if self._buttons[button]:
                self._check_events(EventType.LONG_PRESS, button)

        self._long_press_timers[button] = Timer(timeout, timer_callback)
        self._long_press_timers[button].start()

    def _cancel_long_press_timer(self, button):
        if button in self._long_press_timers:
            self._long_press_timers[button].cancel()


if __name__ == "__main__":
    controller = Controller()
    controller.start()  # Start listening for controller events

    def on_a_button_down(event_type, button):
        if button == ControllerButton.A:
            print(f"{event_type}: A button")

    controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.A, on_a_button_down)
