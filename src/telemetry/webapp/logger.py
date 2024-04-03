import sys
import os
class Loggneer:
    def __init__(self, telem_server):
        self.stdout = sys.stdout
        self.telem_server = telem_server

    def write(self, message):
        if message != '\n':
            self.telem_server.websocket_handler.send_text("logs", message)
        self.stdout.write(message)

    def flush(self):
        self.stdout.flush()

    def isatty(self):
        return False
