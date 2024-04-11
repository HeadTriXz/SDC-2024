import logging


class LoggingHandler(logging.Handler):
    """A class to represent a logging handler."""

    def emit(self, record: logging.LogRecord) -> None:
        """Emit the record.

        :param record: The record.
        """
        print(self.format(record))  # noqa: T201
