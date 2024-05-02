import sys

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from src.telemetry.app import TelemetryServer


class StdoutWrapper:
    """A class to represent a log handler."""

    def __init__(self, telemetry_server: "TelemetryServer") -> None:
        """Initialize the log handler.

        :param telemetry_server: The telemetry server.
        """
        self.stdout = sys.stdout
        self.telemetry_server = telemetry_server

    def write(self, message: str) -> None:
        """Write the message to the log.

        :param: The message to be written.
        """
        if message != "\n":
            self.telemetry_server.websocket_handler.send_text("logs", message)
        self.stdout.write(message)

    def flush(self) -> None:
        """Flush the log."""
        self.stdout.flush()

    def isatty(self) -> bool:
        """Check if the log is a tty."""
        return self.stdout.isatty()

    def __getattr__(self, attr: str) -> Any:
        """Get an attribute.

        :param attr: The attribute to get.
        :return: The attribute.
        """
        return getattr(self.stdout, attr)
