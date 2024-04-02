import asyncio
import base64

import cv2
import numpy as np
from fastapi import WebSocket


class WebsocketDataStream:
    def __init__(self, ws: WebSocket):
        self.ws = ws
        self.sending = True

    async def send_image(self, image: np.ndarray) -> bool:
        if not self.sending:
            return None

        _, buffer = cv2.imencode(".jpg", image)
        image_bytes = base64.b64encode(buffer)
        b64_str = image_bytes.decode("utf-8")
        try:
            await self.ws.send_text(b64_str)
            return True
        except Exception:
            return False

    async def send_text(self, text: str) -> None:
        if not self.sending:
            return None

        try:
            await self.ws.send_text(text)
            return True
        except Exception:
            return False

    async def rec_messages(self) -> None:
        while True:
            data = await self.ws.receive_text()
            if data == "toggle":
                self.sending = not self.sending
                print(f"Sending: {self.sending}")


class WebsocketHandler:
    websocket_clients: dict[str, list[WebsocketDataStream]]

    def __init__(self):
        self.websocket_clients = {}
        self.websockets_active = {}

    def add_socket(self, name: str, websocket):
        if name not in self.websocket_clients:
            self.websocket_clients[name] = []

        self.websocket_clients[name].append(WebsocketDataStream(websocket))
        return self.websocket_clients[name][-1]

    def send_image(self, name: str, image: np.ndarray):
        if name in self.websocket_clients:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            for ws in self.websocket_clients[name]:
                success = loop.run_until_complete(ws.send_image(image))
                if not success and success is not None:
                    self.websocket_clients[name].remove(ws)

    def send_text(self, name: str, text: str):
        if name in self.websocket_clients:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            for ws in self.websocket_clients[name]:
                loop.run_until_complete(ws.send_text(text))

    def remove_socket(self, name: str):
        del self.websocket_clients[name]
