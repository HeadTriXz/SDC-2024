import base64
import cv2
import numpy as np

from asyncio import AbstractEventLoop
from fastapi import WebSocket


class WebsocketDataStream:
    """A class to represent a websocket data stream."""

    def __init__(self, ws: WebSocket, loop: AbstractEventLoop) -> None:
        """Initialize the websocket data stream.

        :param ws: The websocket instance.
        :param loop: The event loop.
        """
        self.ws = ws
        self.sending = True
        self.__loop = loop

    def send_image(self, image: np.ndarray) -> bool | None:
        """Send image to the websocket.

        :param image: The image to be sent.
        :return: True if the image was sent successfully.
        """
        if not self.sending:
            return None

        _, buffer = cv2.imencode(".jpg", image)
        image_bytes = base64.b64encode(buffer)
        b64_str = image_bytes.decode("utf-8")

        return self.send_text(b64_str)

    def send_text(self, text: str) -> bool | None:
        """Send text to the websocket.

        :param text: The text to be sent.
        :return: True if the text was sent successfully.
        """
        if not self.sending:
            return None

        self.__loop.create_task(self.ws.send_text(text))

    async def rec_messages(self) -> None:
        """Receive messages from the websocket."""
        while True:
            data = await self.ws.receive_text()
            if data == "toggle":
                self.sending = not self.sending


class WebsocketHandler:
    """A class to represent a websocket handler."""

    websocket_clients: dict[str, list[WebsocketDataStream]]

    def __init__(self) -> None:
        """Initialize the websocket handler."""
        self.websocket_clients = {}
        self.websockets_active = {}

    def add_socket(self, name: str, websocket: WebSocket, loop: AbstractEventLoop) -> WebsocketDataStream:
        """Add a websocket client to the list of clients.

        :param name: The name of the websocket.
        :param websocket: The websocket instance.
        :param loop: The event loop.
        :return: The websocket data stream.
        """
        if name not in self.websocket_clients:
            self.websocket_clients[name] = []

        self.websocket_clients[name].append(WebsocketDataStream(websocket, loop))
        return self.websocket_clients[name][-1]

    def send_image(self, name: str, image: np.ndarray) -> None:
        """Send image on channel with the given name.

        :param name: The name of the channel.
        :param image: The image to be sent.
        """
        if name in self.websocket_clients:
            for ws in self.websocket_clients[name]:
                ws.send_image(image)

    def send_text(self, name: str, text: str) -> None:
        """Send text on channel with the given name.

        :param name: The name of the channel.
        :param text: The text to be sent.
        """
        if name in self.websocket_clients:
            for ws in self.websocket_clients[name]:
                ws.send_text(text)

    def remove_socket(self, name: str) -> None:
        """Remove a websocket client from the list of clients.

        :param name: The name of the websocket.
        """
        del self.websocket_clients[name]
