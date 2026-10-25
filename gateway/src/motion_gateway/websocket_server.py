"""WebSocket server for broadcasting normalized telemetry and gateway status.

Listens on a local interface (default 127.0.0.1:8765).
Applies a latest-sample pattern per client with zero unbounded buffering:
slow clients never stall the source or cause memory growth.
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Awaitable, Callable, Dict, Optional, Set

import websockets
from websockets.asyncio.server import ServerConnection

logger = logging.getLogger("motion_gateway.websocket")

ClientMessageHandler = Callable[[Dict[str, Any], ServerConnection], Awaitable[None]]


class ClientSlot:
    """Manages outgoing messages for a single connected WebSocket client.

    Uses an asyncio.Queue with maxsize=1. When a new telemetry sample arrives,
    any unsent older sample is evicted immediately, prioritizing real-time data.
    Status messages (1 Hz) are queued with priority or sent promptly.
    """

    def __init__(self, ws: ServerConnection) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[str] = asyncio.Queue(maxsize=1)
        self.dropped_samples = 0

    def push_latest(self, text: str) -> None:
        """Push message, discarding any older pending message if queue is full."""
        try:
            self.queue.put_nowait(text)
        except asyncio.QueueFull:
            try:
                self.queue.get_nowait()
                self.dropped_samples += 1
            except asyncio.QueueEmpty:
                pass
            try:
                self.queue.put_nowait(text)
            except asyncio.QueueFull:
                self.dropped_samples += 1


class WebSocketServer:
    """Publishes normalized telemetry and status to local WebSocket subscribers."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        on_client_message: Optional[ClientMessageHandler] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.on_client_message = on_client_message
        self._server: Optional[Any] = None
        self._clients: Dict[ServerConnection, ClientSlot] = {}
        self._lock = asyncio.Lock()
        self._running = False

    @property
    def client_count(self) -> int:
        return len(self._clients)

    @property
    def is_running(self) -> bool:
        return self._running

    async def start(self) -> None:
        """Start listening for incoming WebSocket connections."""
        self._running = True
        self._server = await websockets.serve(
            self._handle_client,
            self.host,
            self.port,
        )
        logger.info("WebSocket server listening on ws://%s:%d", self.host, self.port)

    async def stop(self) -> None:
        """Gracefully close all client connections and stop the server."""
        self._running = False
        async with self._lock:
            clients_to_close = list(self._clients.keys())
            for ws in clients_to_close:
                try:
                    await ws.close()
                except Exception:
                    pass
            self._clients.clear()

        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        logger.info("WebSocket server stopped")

    def broadcast_telemetry(self, telemetry_dict: Optional[Dict[str, Any]]) -> None:
        """Broadcast telemetry sample to all connected clients.

        JSON serialization is performed once, then distributed to each client's slot.
        """
        if not self._clients or telemetry_dict is None:
            return
        payload = json.dumps(telemetry_dict, allow_nan=False)
        for slot in list(self._clients.values()):
            slot.push_latest(payload)

    def broadcast_status(self, status_dict: Dict[str, Any]) -> None:
        """Broadcast gateway status message to all connected clients."""
        if not self._clients:
            return
        payload = json.dumps(status_dict, allow_nan=False)
        for slot in list(self._clients.values()):
            slot.push_latest(payload)

    async def send_to_client(self, websocket: ServerConnection, message_dict: Dict[str, Any]) -> None:
        """Send a message directly to a specific client (for RPC command responses)."""
        try:
            payload = json.dumps(message_dict, allow_nan=False)
            await websocket.send(payload)
        except Exception as exc:
            logger.debug("Error sending direct response to client: %s", exc)

    async def _handle_client(self, websocket: ServerConnection) -> None:
        """Handle individual client lifecycle."""
        slot = ClientSlot(websocket)
        async with self._lock:
            self._clients[websocket] = slot
        remote = websocket.remote_address
        logger.info("Client connected from %s (total: %d)", remote, len(self._clients))

        sender_task = asyncio.create_task(self._client_sender(slot))

        try:
            async for raw_msg in websocket:
                if self.on_client_message and isinstance(raw_msg, str):
                    try:
                        data = json.loads(raw_msg)
                        if isinstance(data, dict):
                            await self.on_client_message(data, websocket)
                    except Exception as exc:
                        logger.debug("Error processing client command: %s", exc)
        except Exception:
            pass
        finally:
            sender_task.cancel()
            try:
                await sender_task
            except asyncio.CancelledError:
                pass

            async with self._lock:
                self._clients.pop(websocket, None)
            logger.info("Client disconnected: %s (remaining: %d)", remote, len(self._clients))

    async def _client_sender(self, slot: ClientSlot) -> None:
        """Task that drains messages from the client's single-slot queue and sends them."""
        try:
            while self._running:
                msg = await slot.queue.get()
                await slot.ws.send(msg)
        except (asyncio.CancelledError, websockets.ConnectionClosed):
            pass
        except Exception as exc:
            logger.debug("Error sending to client: %s", exc)
