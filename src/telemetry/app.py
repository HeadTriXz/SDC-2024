import fastapi
import logging
import os
import sys
import threading
import uvicorn

from config import config
from starlette.responses import HTMLResponse
from starlette.staticfiles import StaticFiles
from telemetry.data_stream.routes import create_router
from telemetry.update_config.routes import create_config_router
from telemetry.data_stream.websocket_handler import WebsocketHandler
from telemetry.logging_handler import LoggingHandler
from telemetry.stdout_wrapper import StdoutWrapper
from utils.ip_loader import get_ip


def get_path(rel_path: str) -> str:
    """Get the absolute path of the current file.

    :param rel_path: The relative path.
    :return: The absolute path.
    """
    return os.path.join(os.path.dirname(__file__), rel_path)


class TelemetryServer:
    """A class to represent a telemetry server."""

    def __init__(self) -> None:
        """Initialize the telemetry server."""
        self.__port = config.telemetry.server.port
        self.__host = config.telemetry.server.host
        self.thread = threading.Thread(target=self.__start, daemon=True)
        self.__app = fastapi.FastAPI()

        std_wrapper = StdoutWrapper(self)
        sys.stdout = std_wrapper

        logging.basicConfig(level=logging.INFO, handlers=[LoggingHandler()])

        self.websocket_handler = WebsocketHandler()
        self.__app.include_router(create_router(self.websocket_handler))
        self.__app.include_router(create_config_router())

        self.__app.get("/")(self.__index_route)
        self.__app.mount("/js", StaticFiles(directory=get_path("static/js")), name="static")
        self.__app.mount("/css", StaticFiles(directory=get_path("static/css")), name="static")

    def start(self) -> None:
        """Start the telemetry server."""
        if config.telemetry.enabled:
            self.thread.start()

    def __start(self) -> None:
        """Start the telemetry server."""
        uvicorn.run(self.__app, host=self.__host, port=self.__port)

    def __index_route(self) -> HTMLResponse:
        """The index route.

        :return: The HTML response.
        """
        with open(get_path("static/index.html")) as f:
            html = f.read().replace("$root-url", f"{get_ip()}:{self.__port}")
            return HTMLResponse(content=html)
