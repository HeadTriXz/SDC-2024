import logging
import math
from enum import Enum
from threading import Thread, Timer

import inputs
from inputs import get_gamepad

MAX_TRIG_VAL = math.pow(2, 8)
MAX_JOY_VAL = math.pow(2, 15)


class ControllerButton(Enum):
    """Enum for the XBOX controller buttons."""

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
    """Enum for the XBOX controller axis."""

    LS_X = "ABS_X"
    LS_Y = "ABS_Y"
    RS_X = "ABS_RX"
    RS_Y = "ABS_RY"
    DPAD_X = "ABS_HAT0X"
    DPAD_Y = "ABS_HAT0Y"
    RT = "ABS_RZ"
    LT = "ABS_Z"


class EventType(Enum):
    """Enum for the event types."""

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

        self.logger = logging.getLogger(__name__)

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
            except Exception:  # noqa: PERF203, BLE001
                # needed because linux decides to shit itself sometimes
                pass

    def vibrate(self, duration: float = 1000) -> None:
        """Vibrate the controller for a given duration."""
        try:
            self.gamepad.set_vibration(1, 1, duration)
        except Exception as e:  # noqa: BLE001
            self.logger.warning(e)
            self.logger.info("Failed to vibrate")

    def add_listener(
        self, event_type: EventType, button_or_axis: ControllerButton | ControllerAxis, callback: callable
    ) -> None:
        """Add a button or axis listener."""
        if event_type not in EventType:
            raise ValueError(f"Invalid event type: {event_type}")

        if button_or_axis not in ControllerButton and button_or_axis not in ControllerAxis:
            raise ValueError(f"Invalid button or axis: {button_or_axis}")
        key = (event_type, button_or_axis)
        if key not in self._listeners:
            self._listeners[key] = []
        self._listeners[key].append(callback)

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
        if key not in self._listeners:
            return

        if event_type == EventType.AXIS_CHANGED:
            for callback in self._listeners[key]:
                callback(event_type, data, value)
        else:
            for callback in self._listeners[key]:
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
