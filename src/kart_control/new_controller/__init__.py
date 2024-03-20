import math
from enum import Enum
from threading import Thread, Timer
from typing import Any

import inputs
from inputs import get_gamepad

MAX_TRIG_VAL = math.pow(2, 10)
MAX_JOY_VAL = math.pow(2, 15)


def tuple_equals(a: tuple, b: tuple) -> bool:
    """Checks if two tuples are equal based on their hash code.

    :param a: The first tuple to compare.
    :param b: The second tuple to compare with.
    :return: Whether two tuples are equal.
    """
    return hash(a) == hash(b)


def tuple_in_dict(t: tuple, d: dict) -> bool:
    """Check if a tuple is a key in a dictionary.

    :param t: The tuple to check for.
    :param d: The dictionary to check the keys for.
    :return: Whether the tuple is a key in the dictionary.
    """
    return any([tuple_equals(t, key) for key in d.keys()])


def tuple_key_to_value(t: tuple, d: dict) -> Any:
    """Get a value from a dictionary based on a tuple key.

    :param t: The tuple to check for.
    :param d: The dictionary to get the value from.
    :return: The value, if any.
    """
    for key, value in d.items():
        if tuple_equals(t, key):
            return value

    return None


class ControllerButton(Enum):
    """Recognized buttons."""

    A = "BTN_SOUTH"
    B = "BTN_EAST"
    X = "BTN_WEST"
    Y = "BTN_NORTH"
    START = "BTN_START"
    SELECT = "BTN_SELECT"
    LB = "BTN_TL"
    RB = "BTN_TR"
    LS = "BTN_THUMBL"
    RS = "BTN_THUMBR"


class ControllerAxis(Enum):
    """Recognized axis."""

    LS_X = "ABS_X"
    LS_Y = "ABS_Y"
    RS_X = "ABS_RX"
    RS_Y = "ABS_RY"
    DPAD_X = "ABS_HAT0X"
    DPAD_Y = "ABS_HAT0Y"
    RT = "ABS_RZ"
    LT = "ABS_Z"


class EventType(Enum):
    """Types of events to listen on."""

    BUTTON_DOWN = "button_down"
    BUTTON_UP = "button_up"
    LONG_PRESS = "long_press"
    SHORT_PRESS = "short_press"
    AXIS_CHANGED = "axis_changed"


class Controller:
    """A Wrapper around the inputs library to handle the XBOX controller.

    this controller gives an event based interface to the XBOX controller.
    it has the following events
    - BUTTON_DOWN
    - BUTTON_UP
    - LONG_PRESS
    - SHORT_PRESS
    - AXIS_CHANGED

    to use this controller you need to create an instance of it and call the start
    method. after that you can add listeners to the controller to listen to the
    events.

    Example:
    -------
    ```python
    controller = Controller()

    def on_button_down(event, button):
        print(f"Button {button} was pressed")

    def on_axis_changed(event, button, value):
        print(f"Axis {button} was changed to {value}")

    controller.add_listener(EventType.BUTTON_DOWN, ControllerButton.A, on_button_down)
    controller.add_listener(EventType.AXIS_CHANGED, ControllerAxis.LS_X, on_axis_changed)

    """

    def __init__(self) -> None:
        """Create a new controller instance.

        this will only work for a *SINGLE* XBOX controller.
        """
        self.__thread = Thread(target=self.__start, daemon=True)
        self._buttons = {}
        self._axes = {}
        self._listeners = {}
        self._last_buttons = {}
        self._long_press_timers = {}

        self.gamepad = inputs.devices.gamepads[0]

    def start(self) -> None:
        """Start listening to button presses and axis changes."""
        self.__thread.start()

    def __start(self) -> None:
        while True:
            try:
                events = get_gamepad()
                for event in events:
                    if event.ev_type == "Key":
                        self._handle_button_event(event)
                    elif event.ev_type == "Absolute":
                        self._handle_axis_event(event)
            except Exception as e:
                print(e)

    def vibrate(self, duration: int = 1000) -> None:
        """Vibrate the controller.

        Parameters
        ----------
        :param duration int: the duration to vibrate in miliseconds. default = 1000

        """
        try:
            self.gamepad.set_vibration(1, 1, duration)
        except Exception:  # noqa: BLE001
            print("Failed to vibrate")  # noqa: T201

    def add_listener(
        self, event_type: EventType, button_or_axis: ControllerButton | ControllerAxis, callback: callable
    ) -> None:
        """Add a listener to be executed on that event.

        Parameters
        ----------
        :param event_type EventType: the type of event to call it on.
        :param button_or_axis ControllerButton | ControllerAxis: the axis or button that the event needs to be for.
        :param callback callable: the callback to call when the event occurs

        """
        if event_type.value not in EventType:
            raise ValueError(f"Invalid event type: {event_type}")

        if button_or_axis.value not in ControllerButton and button_or_axis.value not in ControllerAxis:
            raise ValueError(f"Invalid button or axis: {button_or_axis}")

        key = (event_type, button_or_axis)
        if not tuple_in_dict(key, self._listeners):
            self._listeners[key] = []

        tuple_key_to_value(key, self._listeners).append(callback)

    def _handle_button_event(self, event: inputs.InputEvent) -> None:
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

    def _handle_axis_event(self, event: inputs.InputEvent) -> None:
        axis = ControllerAxis(event.code)

        if axis in (
                ControllerAxis.LS_X,
                ControllerAxis.LS_Y,
                ControllerAxis.RS_X,
                ControllerAxis.RS_Y,
        ):
            value = event.state / MAX_JOY_VAL
        elif axis not in (ControllerAxis.DPAD_X, ControllerAxis.DPAD_Y):
            value = event.state / MAX_TRIG_VAL
        else:
            value = event.state
        self._axes[axis] = value
        self._check_events(EventType.AXIS_CHANGED, axis, value)

    def _check_events(
            self, event_type: EventType, data: ControllerButton | ControllerAxis, value: float = None
    ) -> None:
        key = (event_type, data)
        if not tuple_in_dict(key, self._listeners):
            return

        if event_type == EventType.AXIS_CHANGED:
            for callback in tuple_key_to_value(key, self._listeners):
                callback(event_type, data, value)
        else:
            for callback in tuple_key_to_value(key, self._listeners):
                callback(event_type, data)

    def _start_long_press_timer(self, button: ControllerButton, timeout: float = 1.5) -> None:
        def timer_callback() -> None:
            if self._buttons[button]:
                self._check_events(EventType.LONG_PRESS, button)

        self._long_press_timers[button] = Timer(timeout, timer_callback)
        self._long_press_timers[button].start()

    def _cancel_long_press_timer(self, button: ControllerButton) -> None:
        if button in self._long_press_timers:
            self._long_press_timers[button].cancel()
