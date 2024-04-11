from fastapi import APIRouter, WebSocket
from telemetry.data_stream.websocket_handler import WebsocketHandler
from starlette.websockets import WebSocketDisconnect


def create_router(websocket_handler: WebsocketHandler) -> APIRouter:
    """Create a router for the websocket.

    :param websocket_handler: The websocket handler.
    :return: The router.
    """
    router = APIRouter()

    @router.websocket("/ws/{name}")
    async def websocket_endpoint(name: str, websocket: WebSocket) -> None:
        """Create a websocket endpoint.

        :param name: The name of the websocket.
        :param websocket: The websocket instance.
        """
        await websocket.accept()
        client = websocket_handler.add_socket(name, websocket)
        try:
            await client.rec_messages()
        except WebSocketDisconnect:
            websocket_handler.websocket_clients[name].remove(client)

    return router
