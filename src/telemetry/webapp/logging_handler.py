import logging

class LoggingHandler(logging.Handler):
    """A class to represent a logging handler."""

    def __init__(self) -> None:
        """Initialize the logging handler.

        :param server: The server.
        """
        super().__init__()

    def emit(self, record: logging.LogRecord) -> None:
        """Emit the record.

        :param record: The record.
        """
        print(self.format(record))
