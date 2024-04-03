from fastapi import APIRouter, WebSocket
from telemetry.webapp.data_stream.websocket_handler import WebsocketHandler


def create_router(websocket_handler: WebsocketHandler) -> APIRouter:
    """Create a FastAPI router for the data stream."""
    router = APIRouter()

    @router.websocket("/ws/{name}")
    async def websocket_endpoint(name: str, websocket: WebSocket) -> None:
        """Create a websocket endpoint."""
        await websocket.accept()
        client = websocket_handler.add_socket(name, websocket)
        await client.rec_messages()

    return router
