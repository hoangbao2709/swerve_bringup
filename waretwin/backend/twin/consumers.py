import logging
import asyncio
import time
from urllib.parse import parse_qs
from channels.generic.websocket import AsyncJsonWebsocketConsumer
from .runtime import runtime
from .control_timing import profile_async
from .realtime_consumer import RealtimeDispatchMixin
from .visualization_outbox import VisualizationOutbox
from .operator_security import authorize_websocket_message

log = logging.getLogger(__name__)

class TwinConsumer(RealtimeDispatchMixin, AsyncJsonWebsocketConsumer):
    database_free_types = frozenset({'MANUAL_ACQUIRE', 'ROBOT_MANUAL', 'ROBOT_MODE', 'ROBOT_DETAIL_VIEW', 'ROBOT_DETAIL_FRAME_RECEIVED'})

    async def connect(self):
        query = parse_qs(self.scope.get('query_string', b'').decode())
        self.control_only = (query.get('control_only') or ['0'])[0] == '1'
        self._message_window_started = time.monotonic()
        self._message_window_count = 0
        self._manual_window_started = self._message_window_started
        self._manual_window_count = 0
        self.operator = self.scope.get('waretwin_operator')
        self._operator_expiry_task = None
        if self.operator and self.operator.get('expires_at') != float('inf'):
            self._operator_expiry_task = asyncio.create_task(self._close_at_operator_expiry())
        if not self.control_only:
            await self.channel_layer.group_add('twin_clients', self.channel_name)
        await self.accept()
        if self.control_only:
            await self.send_json({'type': 'ROBOT_MANUAL_CHANNEL_READY'})
            return
        self.visualization_outbox = VisualizationOutbox(self.send)
        runtime.visualization_clients.add(self)
        runtime.client_count += 1
        await runtime.ensure_started()
        await self.send_json(runtime.full_message())
        await self.send_json(runtime.runtime_status_message())
        for snapshot in runtime.robot_runtime_map_snapshots.values():
            await self.send_json(snapshot)
        for snapshot in runtime.robot_slam_map_snapshots.values():
            await self.send_json(snapshot)
        for floor, values in runtime.engine.traffic.items():
            await self.send_json({'type': 'HEATMAP', 'layer': runtime.heatmap_layer('CONGESTION', values, floor)})

    async def disconnect(self, close_code):
        try:
            expiry_task = getattr(self, '_operator_expiry_task', None)
            if expiry_task and expiry_task is not asyncio.current_task():
                expiry_task.cancel()
            owners = getattr(runtime, 'manual_owners', {})
            for robot_id, owner in list(owners.items()):
                owner_channel = owner.get('channel_name') if isinstance(owner, dict) else owner
                if owner_channel == self.channel_name:
                    owners.pop(robot_id, None)
                    await runtime.gateway().send_command(robot_id, 'MANUAL_DISCONNECT', {})
            if not getattr(self, 'control_only', False):
                await self.channel_layer.group_discard('twin_clients', self.channel_name)
        finally:
            if not getattr(self, 'control_only', False):
                runtime.visualization_clients.discard(self)
                if hasattr(self, 'visualization_outbox'):
                    await self.visualization_outbox.close()
                runtime.client_count = max(0, runtime.client_count - 1)

    async def _close_at_operator_expiry(self):
        expires_at = float(self.operator.get('expires_at', 0))
        await asyncio.sleep(max(0.0, expires_at - time.time()))
        await self.close(code=4401)

    async def receive_json(self, content, **kwargs):
        if not isinstance(content, dict):
            await self.send_json({'type': 'ERROR', 'code': 'BAD_MESSAGE', 'message': 'message must be a JSON object'})
            return
        # Keep direct ASGI dispatch and upgraded in-flight consumers safe if a
        # middleware/test harness invokes a frame before connect() initialized
        # the rate-limit windows.
        now = time.monotonic()
        if not hasattr(self, '_manual_window_started'):
            self._manual_window_started = now
            self._manual_window_count = 0
        if not hasattr(self, '_message_window_started'):
            self._message_window_started = now
            self._message_window_count = 0
        if getattr(self, 'control_only', False) and content.get('type') not in ('MANUAL_ACQUIRE', 'ROBOT_MANUAL'):
            await self.send_json({'type': 'ERROR', 'code': 'CONTROL_CHANNEL_RESTRICTED',
                                  'message': 'manual-only channel accepts manual acquire/command frames'})
            return
        if not authorize_websocket_message(getattr(self, 'operator', None), content):
            await self.send_json({'type': 'ERROR', 'code': 'OPERATOR_PERMISSION_REQUIRED',
                                  'message': 'operator identity lacks permission or robot scope for this command'})
            return
        if content.get('type') in ('MANUAL_ACQUIRE', 'ROBOT_MANUAL'):
            manual_now = time.monotonic()
            if manual_now - self._manual_window_started >= 1.0:
                self._manual_window_started = manual_now
                self._manual_window_count = 0
            self._manual_window_count += 1
            if self._manual_window_count > 25:
                await self.send_json({'type': 'ERROR', 'code': 'MANUAL_RATE_LIMITED',
                    'message': 'manual command rate exceeded; release control and retry'})
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
            if content.get('type') == 'ROBOT_DETAIL_FRAME_RECEIVED':
                delivery_id = content.get('delivery_id')
                if type(delivery_id) is int and delivery_id > 0:
                    self.visualization_outbox.acknowledge(delivery_id)
                return
            if content.get('type') == 'ROBOT_MANUAL':
                content = dict(content, _consumer_monotonic=time.monotonic())
            elif content.get('type') == 'ROBOT_DETAIL_VIEW':
                self.visualization_outbox.delivery_ack = content.get('delivery_ack') is True
                content = dict(content, _view_received_ms=time.time() * 1000)
            if content.get('type') in ('MANUAL_ACQUIRE', 'ROBOT_MANUAL', 'ROBOT_MODE',
                                       'NAV_GOAL', 'NAV_PAUSE', 'NAV_RESUME', 'NAV_CANCEL'):
                log.info('authorized WebSocket operator operation identity=%s type=%s robot=%s',
                         (getattr(self, 'operator', None) or {}).get('identity', 'local-loopback'),
                         content.get('type'), content.get('robot_id'))
            await runtime.handle_message(self, content)
        except Exception as exc:
            # Keep a malformed/failed command isolated to this frame. The
            # runtime loop and other clients must remain available.
            log.exception('frontend WebSocket message failed', extra={'message_type': str(content.get('type') or 'UNKNOWN')})
            await self.send_json({'type': 'ERROR', 'code': 'INTERNAL_ERROR', 'message': f'command failed: {type(exc).__name__}'})

    @profile_async
    async def twin_message(self, event):
        await self.send_json(event['payload'])
