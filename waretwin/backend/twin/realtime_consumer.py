"""Keep database-free control frames independent of the shared ORM executor."""
from .control_timing import DispatchTimingMixin


class RealtimeDispatchMixin(DispatchTimingMixin):
    database_free_types = frozenset()

    async def dispatch(self, message):
        if message.get('type') == 'twin.message':
            # This handler only forwards an already-authorized runtime event.
            return await self.twin_message(message)
        if message.get('type') == 'websocket.receive' and message.get('text'):
            try:
                content = await self.decode_json(message['text'])
            except (ValueError, TypeError):
                # Preserve the framework's error and connection handling.
                return await super().dispatch(message)
            kind = content.get('type') if isinstance(content, dict) else None
            if isinstance(kind, str) and kind in self.database_free_types:
                # Run the normal rate, schema, authorization, ownership and
                # map/safety validation. Only the unused ORM cleanup wait is
                # omitted. Connect/auth and DB-using messages retain cleanup.
                return await self.receive_json(content)
        return await super().dispatch(message)
