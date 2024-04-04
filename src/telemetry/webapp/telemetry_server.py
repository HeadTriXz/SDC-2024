import os
import threading
import logging
from starlette.responses import HTMLResponse
from starlette.staticfiles import StaticFiles
import sys
from config import config

import fastapi
import uvicorn

from telemetry.webapp.data_stream.routes import create_router
from telemetry.webapp.data_stream.websocket_handler import WebsocketHandler
from telemetry.webapp.logger import Loghandler

def get_path(rel_path: str) -> str:
    """Get the absolute path of the current file."""
    return os.path.join(os.path.dirname(__file__), rel_path)


class TelemetryServer:
    """A class to represent a telemetry server."""

    def __init__(self) -> None:
        """Initialize the telemetry server."""
        self.__port = config.telemetry.server.port
        self.__host = config.telemetry.server.host
        self.thread = threading.Thread(target=self.__start, daemon=True)
        self.__app = fastapi.FastAPI()

        logger = Loghandler(self)
        sys.stdout = logger
        logging.basicConfig(level=logging.INFO, stream=logger)

        self.websocket_handler = WebsocketHandler()
        self.__app.include_router(create_router(self.websocket_handler))

        self.__app.get("/")(self.__index_route)
        self.__app.mount("/js", StaticFiles(directory=get_path("../../../resources/webcontent/js")), name="static")
        self.__app.mount("/css", StaticFiles(directory=get_path("../../../resources/webcontent/css")), name="static")

    def start(self) -> None:
        """Start the telemetry server."""
        self.thread.start()

    def __start(self) -> None:
        """Start the telemetry server."""
        uvicorn.run(self.__app, host=self.__host, port=self.__port)

    @staticmethod
    def __index_route() -> HTMLResponse:
        """The index route."""
        with open(get_path("../../../resources/webcontent/index.html")) as f:
            return HTMLResponse(content=f.read())
