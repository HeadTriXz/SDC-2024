from fastapi import APIRouter, WebSocket

from telemetry.webapp.data_stream.websocket_handler import WebsocketHandler


class ConnectionHandler:
    def __init__(self):
        self.clients = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.clients.append(websocket)

    async def send_message(self, message: str):
        for client in self.clients:
            await client.send_text(message)

    async def disconnect(self, websocket: WebSocket):
        self.clients.remove(websocket)


def create_router(websocket_handler: WebsocketHandler) -> APIRouter:
    router = APIRouter()

    @router.websocket("/ws/{name}")
    async def websocket_endpoint(name: str, websocket: WebSocket):
        print(f"Connected to {name}")
        await websocket.accept()
        client = websocket_handler.add_socket(name, websocket)
        await client.rec_messages()

    return router
