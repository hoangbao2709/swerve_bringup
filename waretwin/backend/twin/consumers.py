from urllib.parse import parse_qs
from channels.db import database_sync_to_async
from channels.generic.websocket import AsyncJsonWebsocketConsumer
from .auth import user_from_token
from .runtime import runtime

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
        runtime.client_count += 1
        await runtime.ensure_started()
        await self.send_json(runtime.full_message())
        await self.send_json(runtime.runtime_status_message())
        for floor, values in runtime.engine.traffic.items():
            await self.send_json({'type': 'HEATMAP', 'layer': runtime.heatmap_layer('CONGESTION', values, floor)})

    async def disconnect(self, close_code):
        try:
            await self.channel_layer.group_discard('twin_clients', self.channel_name)
        finally:
            runtime.client_count = max(0, runtime.client_count - 1)

    async def receive_json(self, content, **kwargs):
        await runtime.handle_message(self, content, self.scope.get('waretwin_user'))

    async def twin_message(self, event):
        await self.send_json(event['payload'])
