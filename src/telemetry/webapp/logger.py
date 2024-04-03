import sys


class Loghandler:
    def __init__(self, telemetry_server):
        self.stdout = sys.stdout
        self.telemetry_server = telemetry_server

    def write(self, message):
        if message != '\n':
            self.telemetry_server.websocket_handler.send_text("logs", message)
        self.stdout.write(message)

    def flush(self):
        self.stdout.flush()

    def isatty(self):
        return False
