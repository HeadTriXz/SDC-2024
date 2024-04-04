import sys

from telemetry.webapp.telemetry_server import TelemetryServer


class Loghandler:
    """A class to represent a log handler."""

    def __init__(self, telemetry_server: TelemetryServer) -> None:
        """Initialize the log handler."""
        self.stdout = sys.stdout
        self.telemetry_server = telemetry_server

    def write(self, message: str) -> None:
        """Write the message to the log."""
        if message != "\n":
            self.telemetry_server.websocket_handler.send_text("logs", message)
        self.stdout.write(message)

    def flush(self) -> None:
        """Flush the log."""
        self.stdout.flush()

    def isatty(self) -> bool:
        """Check if the log is a tty."""
        return False
