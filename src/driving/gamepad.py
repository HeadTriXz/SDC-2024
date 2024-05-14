import inputs
import logging
import math
import time

from enum import Enum
from threading import Thread, Timer


MAX_TRIG_VAL = math.pow(2, 8)
MAX_JOY_VAL = math.pow(2, 15)


class GamepadButton(str, Enum):
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


class GamepadAxis(str, Enum):
    """Recognized axis."""

    LS_X = "ABS_X"
    LS_Y = "ABS_Y"
    RS_X = "ABS_RX"
    RS_Y = "ABS_RY"
    DPAD_X = "ABS_HAT0X"
    DPAD_Y = "ABS_HAT0Y"
    RT = "ABS_RZ"
    LT = "ABS_Z"


class EventType(str, Enum):
    """Types of events to listen on."""

    BUTTON_DOWN = "button_down"
    BUTTON_UP = "button_up"
    LONG_PRESS = "long_press"
    SHORT_PRESS = "short_press"
    AXIS_CHANGED = "axis_changed"


class Gamepad:
    """A class to handle gamepad inputs.

    Attributes
    ----------
        gamepad (inputs.GamePad): The gamepad to use.

    """

    gamepad: inputs.GamePad

    def __init__(self) -> None:
        """Initialize the controller."""
        if len(inputs.devices.gamepads) == 0:
            raise Exception("There are no gamepads connected")

        self.gamepad = inputs.devices.gamepads[0]

        self._buttons = {}
        self._axes = {}
        self._listeners = {}
        self._last_buttons = {}
        self._long_press_timers = {}
        self.__thread = Thread(target=self.__listen, daemon=True)

    def add_listener(
            self,
            input_type: GamepadButton | GamepadAxis,
            event_type: EventType,
            callback: callable
    ) -> None:
        """Add a listener for an event.

        :param input_type: The type of input to listen on.
        :param event_type: The type of event to listen for.
        :param callback: The callback to call when the event occurs.
        """
        if input_type not in self._listeners:
            self._listeners[input_type] = {}

        if event_type not in self._listeners[input_type]:
            self._listeners[input_type][event_type] = []

        self._listeners[input_type][event_type].append(callback)

    def start(self) -> None:
        """Start listening for gamepad events."""
        self.__thread.start()

    def vibrate(self, duration: int = 1000, block: bool = True) -> None:
        """Vibrate the gamepad.

        :param duration: The duration to vibrate in milliseconds.
        :param block: Whether to block until the vibration is done.
        """
        try:
            self.gamepad.set_vibration(1, 1, duration)
            if block:
                time.sleep(duration / 1000)
                self.gamepad.set_vibration(0, 0, 0)
        except NotImplementedError:
            logging.warning("Tried vibrating on an unsupported device")
        except Exception as e:
            logging.error("Failed to vibrate: %s", e)

    def _cancel_long_press_timer(self, button: GamepadButton) -> None:
        """Cancel the long press timer for a button.

        :param button: The button to cancel the timer for.
        """
        if button in self._long_press_timers:
            self._long_press_timers[button].cancel()

    def _check_events(
            self,
            input_type: GamepadButton | GamepadAxis,
            event_type: EventType,
            value: float = None
    ) -> None:
        """Check if any listeners are waiting for an event.

        :param input_type: The type of input to check for.
        :param event_type: The type of event to check for.
        :param value: The value of the input.
        """
        if input_type not in self._listeners:
            return

        if event_type not in self._listeners[input_type]:
            return

        for callback in self._listeners[input_type][event_type]:
            callback(input_type, event_type, value)

    def _handle_axis_event(self, event: inputs.InputEvent) -> None:
        """Handle an axis event.

        :param event: The event to handle.
        """
        axis = GamepadAxis(event.code)

        value = event.state
        if axis in [GamepadAxis.LS_X, GamepadAxis.LS_Y, GamepadAxis.RS_X, GamepadAxis.RS_Y]:
            value /= MAX_JOY_VAL
        elif axis not in [GamepadAxis.DPAD_X, GamepadAxis.DPAD_Y]:
            value /= MAX_TRIG_VAL

        self._axes[axis] = value
        self._check_events(axis, EventType.AXIS_CHANGED, value)

    def _handle_button_event(self, event: inputs.InputEvent) -> None:
        """Handle a button event.

        :param event: The event to handle.
        """
        button = GamepadButton(event.code)

        if event.state:
            self._buttons[button] = True
            self._check_events(button, EventType.BUTTON_DOWN)
            if button not in self._last_buttons or not self._last_buttons[button]:
                self._start_long_press_timer(button)
        else:
            self._buttons[button] = False
            self._check_events(button, EventType.BUTTON_UP)
            self._cancel_long_press_timer(button)
        self._last_buttons[button] = event.state

    def _start_long_press_timer(self, button: GamepadButton, timeout: float = 1.5) -> None:
        """Start a timer for a long press event.

        :param button: The button to start the timer for.
        :param timeout: The time to wait before the event occurs.
        """
        def timer_callback() -> None:
            if self._buttons[button]:
                self._check_events(button, EventType.LONG_PRESS)

        self._long_press_timers[button] = Timer(timeout, timer_callback)
        self._long_press_timers[button].start()

    def __listen(self) -> None:
        """Listen for gamepad events."""
        while True:
            try:
                events = inputs.get_gamepad()
                for event in events:
                    if event.ev_type == "Key":
                        self._handle_button_event(event)
                    elif event.ev_type == "Absolute":
                        self._handle_axis_event(event)
            except (inputs.UnknownEventType, inputs.UnknownEventCode):
                pass
            except (inputs.UnpluggedError, OSError):
                # The 'inputs' library does not support hot-plugging for Linux.
                self.gamepad._character_file = None
                self.gamepad._write_file = None

                logging.warning("Gamepad was unplugged. Please reconnect it.")
                time.sleep(1)
            except Exception as e:
                logging.error("Failed to read gamepad events: %s", e)
