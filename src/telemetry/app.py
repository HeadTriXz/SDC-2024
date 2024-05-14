import fastapi
import os
import sys
import threading
import uvicorn

from fastapi import HTTPException
from starlette.responses import HTMLResponse
from starlette.staticfiles import StaticFiles
from typing import Any

from src.config import config
from src.telemetry.data_stream.routes import create_router
from src.telemetry.data_stream.websocket_handler import WebsocketHandler
from src.telemetry.file_io_wrapper import FileIOWrapper
from src.telemetry.update_config.routes import create_config_router
from src.utils.ip_loader import get_ip


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

        sys.stdout = FileIOWrapper(self, sys.stdout)
        sys.stderr = FileIOWrapper(self, sys.stderr)

        self.available_functions = {}

        self.websocket_handler = WebsocketHandler()
        self.__app.include_router(create_router(self.websocket_handler))
        self.__app.include_router(create_config_router())

        self.__app.get("/")(self.__index_route)
        self.__app.post("/execute_function/{name}")(self.__execute_function)
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

    def add_callback_function(self, name: str, func: callable) -> None:
        """Add a callback function.

        :param name: The name of the callback function.
        :param func: The callback function to add.
        """
        self.available_functions[name] = func

    def __execute_function(self, name: str) -> Any:
        """Execute a function based on the provided name.

        :param name: The name of the function to execute.
        :return: The result of the executed function.
        """
        if name in self.available_functions:
            return self.available_functions[name]()

        raise HTTPException(status_code=404, detail="Function not found")
