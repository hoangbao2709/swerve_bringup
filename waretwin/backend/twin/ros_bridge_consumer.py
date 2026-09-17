from __future__ import annotations

from urllib.parse import parse_qs

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from django.conf import settings

from .runtime import runtime


class RosBridgeRegistry:
    """One authenticated ROS bridge connection used by the gateway."""

    def __init__(self) -> None:
        self.consumer: RosBridgeConsumer | None = None

    async def send(self, payload: dict) -> bool:
        if self.consumer is None:
            return False
        try:
            await self.consumer.send_json(payload)
            return True
        except Exception:
            return False


registry = RosBridgeRegistry()


class RosBridgeConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        query = parse_qs(self.scope.get('query_string', b'').decode())
        token = (query.get('token') or [None])[0]
        expected = str(getattr(settings, 'WARETWIN_ROS_BRIDGE_TOKEN', '') or '')
        if not expected or token != expected:
            await self.close(code=4403)
            return
        if registry.consumer is not None and registry.consumer is not self:
            await registry.consumer.close(code=4002)
        registry.consumer = self
        await self.accept()
        await runtime.bridge_connected()

    async def disconnect(self, close_code):
        if registry.consumer is self:
            registry.consumer = None
            await runtime.bridge_disconnected()

    async def receive_json(self, content, **kwargs):
        if not isinstance(content, dict):
            return
        await runtime.handle_ros_message(content)
