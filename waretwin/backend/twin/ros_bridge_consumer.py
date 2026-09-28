from __future__ import annotations

import logging
import time
from urllib.parse import parse_qs

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from django.conf import settings

from .runtime import runtime

log = logging.getLogger(__name__)


class RosBridgeRegistry:
    """Authenticated ROS bridge connections keyed by robot identity.

    ``consumer`` remains a compatibility property for older single-robot tests
    and integrations; new commands are routed by ``payload.robot_id``.
    """

    def __init__(self) -> None:
        self.consumers: dict[str, RosBridgeConsumer] = {}

    @property
    def consumer(self):
        return next(iter(self.consumers.values()), None)

    @consumer.setter
    def consumer(self, value):
        # Preserve the old test/integration seam while storing connections in
        # the same keyed registry as real multi-robot bridges.
        if value is None:
            self.consumers.clear()
            return
        robot_id = str(getattr(value, 'robot_id', None) or 'R01')
        self.consumers[robot_id] = value

    async def send(self, payload: dict) -> bool:
        robot_id = str(payload.get('robot_id') or '').strip()
        if robot_id:
            targets = [self.consumers[robot_id]] if robot_id in self.consumers else []
        else:
            targets = list(self.consumers.values())
        if not targets:
            return False
        sent = False
        for consumer in targets:
            try:
                await consumer.send_json(payload)
                sent = True
            except Exception:
                log.exception('ROS bridge send failed', extra={
                    'message_type': payload.get('type', 'UNKNOWN'),
                    'robot_id': getattr(consumer, 'robot_id', robot_id),
                })
        return sent


registry = RosBridgeRegistry()


class RosBridgeConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        query = parse_qs(self.scope.get('query_string', b'').decode())
        token = (query.get('token') or [None])[0]
        expected = str(getattr(settings, 'WARETWIN_ROS_BRIDGE_TOKEN', '') or '')
        if not expected or token != expected:
            log.warning('ROS bridge authentication rejected')
            await self.close(code=4403)
            return
        self.robot_id = str((query.get('robot_id') or ['R01'])[0] or 'R01').strip()
        if not self.robot_id or len(self.robot_id) > 64:
            await self.close(code=4400)
            return
        previous = registry.consumers.get(self.robot_id)
        if previous is not None and previous is not self:
            try:
                await previous.close(code=4002)
            except Exception:
                log.exception('failed to close previous ROS bridge connection', extra={'robot_id': self.robot_id})
        registry.consumers[self.robot_id] = self
        self._message_window_started = time.monotonic()
        self._message_window_count = 0
        await self.accept()
        try:
            await runtime.bridge_connected(self.robot_id)
        except Exception as exc:
            log.exception('ROS bridge connect initialization failed')
            try:
                await self.send_json({'type': 'BRIDGE_ERROR', 'code': 'CONNECT_INIT_ERROR', 'message': type(exc).__name__})
            except Exception:
                log.exception('failed to send ROS bridge connect error')
        log.info('ROS bridge connected')

    async def disconnect(self, close_code):
        if registry.consumers.get(getattr(self, 'robot_id', '')) is self:
            registry.consumers.pop(self.robot_id, None)
            try:
                await runtime.bridge_disconnected(self.robot_id)
            except Exception:
                log.exception('ROS bridge disconnect cleanup failed')
            log.info('ROS bridge disconnected', extra={'close_code': close_code})

    async def receive_json(self, content, **kwargs):
        if not isinstance(content, dict):
            log.warning('ROS bridge sent a non-object message')
            return
        now = time.monotonic()
        if now - self._message_window_started >= 1.0:
            self._message_window_started = now
            self._message_window_count = 0
        self._message_window_count += 1
        if self._message_window_count > 500:
            log.warning('ROS bridge message rate limit exceeded')
            await self.send_json({'type': 'BRIDGE_ERROR', 'code': 'RATE_LIMITED', 'message': 'message rate limit exceeded'})
            return
        message_type = str(content.get('type') or 'UNKNOWN').upper()
        reported_robot_id = str(content.get('robot_id') or '').strip()
        if reported_robot_id and reported_robot_id != self.robot_id:
            log.warning('ROS bridge sent telemetry for another robot', extra={
                'bridge_robot_id': self.robot_id,
                'reported_robot_id': reported_robot_id,
                'message_type': message_type,
            })
            await self.send_json({
                'type': 'BRIDGE_ERROR', 'code': 'ROBOT_ID_MISMATCH',
                'message': 'bridge messages must use the authenticated robot_id',
            })
            return
        content = {**content, 'robot_id': self.robot_id}
        try:
            await runtime.handle_ros_message(content)
        except Exception as exc:
            # A malformed telemetry/status packet must not tear down the only
            # bridge connection or make the runtime silently lose diagnostics.
            log.exception('ROS bridge message handler failed', extra={'message_type': message_type})
            try:
                await self.send_json({
                    'type': 'BRIDGE_ERROR',
                    'code': 'MESSAGE_HANDLER_ERROR',
                    'message': f'failed to process {message_type}: {type(exc).__name__}',
                })
            except Exception:
                log.exception('failed to send ROS bridge message error')
