import logging
import time
from urllib.parse import parse_qs
from channels.db import database_sync_to_async
from channels.generic.websocket import AsyncJsonWebsocketConsumer
from .auth import user_from_token
from .runtime import runtime

log = logging.getLogger(__name__)

@database_sync_to_async
def resolve_user(token: str | None):
    return user_from_token(token)

class TwinConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        query = parse_qs(self.scope.get('query_string', b'').decode())
        token = (query.get('token') or [None])[0]
        user = await resolve_user(token)
        if user is None:
            await self.close(code=4401)
            return
        self.scope['waretwin_user'] = user
        await self.channel_layer.group_add('twin_clients', self.channel_name)
        await self.accept()
        self._message_window_started = time.monotonic()
        self._message_window_count = 0
        runtime.client_count += 1
        await runtime.ensure_started()
        await self.send_json(runtime.full_message())
        await self.send_json(runtime.runtime_status_message())
        for snapshot in runtime.robot_map_snapshots.values():
            await self.send_json(snapshot)
        for floor, values in runtime.engine.traffic.items():
            await self.send_json({'type': 'HEATMAP', 'layer': runtime.heatmap_layer('CONGESTION', values, floor)})

    async def disconnect(self, close_code):
        try:
            await self.channel_layer.group_discard('twin_clients', self.channel_name)
        finally:
            runtime.client_count = max(0, runtime.client_count - 1)

    async def receive_json(self, content, **kwargs):
        if not isinstance(content, dict):
            await self.send_json({'type': 'ERROR', 'code': 'BAD_MESSAGE', 'message': 'message must be a JSON object'})
            return
        now = time.monotonic()
        if now - self._message_window_started >= 1.0:
            self._message_window_started = now
            self._message_window_count = 0
        self._message_window_count += 1
        if self._message_window_count > 100:
            await self.send_json({'type': 'ERROR', 'code': 'RATE_LIMITED', 'message': 'client message rate limit exceeded'})
            return
        try:
            await runtime.handle_message(self, content, self.scope.get('waretwin_user'))
        except Exception as exc:
            # Keep a malformed/failed command isolated to this frame. The
            # runtime loop and other clients must remain available.
            log.exception('frontend WebSocket message failed', extra={'message_type': str(content.get('type') or 'UNKNOWN')})
            await self.send_json({'type': 'ERROR', 'code': 'INTERNAL_ERROR', 'message': f'command failed: {type(exc).__name__}'})

    async def twin_message(self, event):
        await self.send_json(event['payload'])
