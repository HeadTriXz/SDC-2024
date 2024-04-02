import os
import threading

from starlette.responses import HTMLResponse
from starlette.staticfiles import StaticFiles

from config import config

import fastapi
import uvicorn

from telemetry.webapp.data_stream.routes import create_router
from telemetry.webapp.data_stream.websocket_handler import WebsocketHandler


def get_path(rel_path: str) -> str:
    return os.path.join(os.path.dirname(__file__), rel_path)


class TelemetryServer:
    """A class to represent a telemetry server."""

    def __init__(self):
        self.__port = config.telemetry.server.port
        self.__host = config.telemetry.server.host
        self.thread = threading.Thread(target=self.__start, daemon=True)
        self.__app = fastapi.FastAPI()

        self.websocket_handler = WebsocketHandler()
        self.__app.include_router(create_router(self.websocket_handler))

        self.__app.get("/")(self.__index_route)
        self.__app.mount("/js", StaticFiles(directory=get_path("../../../resources/webcontent/js")), name="static")
        self.__app.mount("/css", StaticFiles(directory=get_path("../../../resources/webcontent/css")), name="static")

    def start(self):
        self.thread.start()

    def __start(self):
        uvicorn.run(self.__app, host=self.__host, port=self.__port)

    @staticmethod
    def __index_route():
        return HTMLResponse(content=open(get_path("../../../resources/webcontent/index.html")).read())
