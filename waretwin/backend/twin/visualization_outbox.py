"""Bounded latest-only visualization lane per connected Web consumer."""
import asyncio
import json
import time

VISUALIZATION_TYPES = frozenset({'ROBOT_DETAIL_VIEW_STATUS', 'MAP_SNAPSHOT',
    'LIDAR_SCAN', 'LIDAR_MAP_2D', 'LIDAR_MAP_3D', 'LIDAR_STREAM_DIAGNOSTICS',
    'RUNTIME_STATUS', 'COMMAND_DIAGNOSTICS', 'SYSTEM_DIAGNOSTICS', 'CONTROLLER_STATE'})
FRAME_TYPES = frozenset({'MAP_SNAPSHOT', 'LIDAR_SCAN', 'LIDAR_MAP_2D', 'LIDAR_MAP_3D'})


class VisualizationOutbox:
    def __init__(self, send, capacity=16):
        self.send = send
        self.capacity = capacity
        self.pending = {}
        self.applied = {}
        self.dropped = 0
        self.wake = asyncio.Event()
        self.delivery_ack = False
        self.delivery_id = 0
        self.inflight = None
        self.received = asyncio.Event()
        self.task = asyncio.create_task(self.run())

    def acknowledge(self, delivery_id):
        if delivery_id == self.inflight:
            self.inflight = None
            self.received.set()

    def offer(self, payload):
        kind = payload['type']
        rid = str(payload.get('robot_id') or (payload.get('map') or payload.get('scan') or payload.get('controller') or {}).get('robot_id') or '')
        key = (kind, rid)
        if kind == 'ROBOT_DETAIL_VIEW_STATUS':
            self.applied[rid] = (payload.get('bridge_epoch'), payload.get('view_epoch'))
            if len(self.applied) > 64:
                self.applied.pop(next(iter(self.applied)))
            for old in list(self.pending):
                if old[1] == rid and old[0] in ('LIDAR_MAP_2D', 'LIDAR_MAP_3D', 'LIDAR_SCAN'):
                    self.pending.pop(old); self.dropped += 1
        if key in self.pending:
            self.dropped += 1
        self.pending[key] = payload
        while len(self.pending) > self.capacity:
            oldest = next((item for item in self.pending if item[0] != 'ROBOT_DETAIL_VIEW_STATUS'), next(iter(self.pending)))
            self.pending.pop(oldest); self.dropped += 1
        self.wake.set()

    def current(self, payload):
        if payload['type'] not in ('LIDAR_MAP_2D', 'LIDAR_MAP_3D'):
            return True
        applied = self.applied.get(payload.get('robot_id'))
        return applied is None or applied == (payload.get('bridge_epoch'), payload.get('view_epoch'))

    async def run(self):
        while True:
            await self.wake.wait()
            while self.pending:
                eligible = [key for key in self.pending if not (
                    self.delivery_ack and self.inflight is not None and key[0] in FRAME_TYPES)]
                key = next((key for key in eligible if key[0] == 'ROBOT_DETAIL_VIEW_STATUS'),
                    eligible[0] if eligible else next(iter(self.pending)))
                payload = self.pending.pop(key)
                if not self.current(payload):
                    self.dropped += 1
                    continue
                # ASGI send only enqueues bytes. With opt-in browser receipts,
                # at most one disposable frame can be in the transport queue.
                # Lightweight view ACKs never wait for a heavy frame receipt.
                if self.delivery_ack and key[0] in FRAME_TYPES and self.inflight is not None:
                    self.pending[key] = payload
                    self.wake.clear()
                    ack_wait = asyncio.create_task(self.received.wait())
                    view_wait = asyncio.create_task(self.wake.wait())
                    try:
                        await asyncio.wait([ack_wait, view_wait], timeout=5,
                            return_when=asyncio.FIRST_COMPLETED)
                    finally:
                        ack_wait.cancel(); view_wait.cancel()
                        await asyncio.gather(ack_wait, view_wait, return_exceptions=True)
                    self.received.clear()
                    # Do not add more frames behind a stalled browser. Retain
                    # latest pending references until a receipt or disconnect.
                    continue
                if self.delivery_ack and key[0] in FRAME_TYPES:
                    self.delivery_id += 1
                    self.inflight = self.delivery_id
                    payload = {**payload, 'visualization_delivery_id': self.delivery_id}
                if isinstance(payload.get('view_timing'), dict):
                    payload = {**payload, 'view_timing': {**payload['view_timing'],
                        'django_browser_send_ms': time.time() * 1000,
                        'django_visualization_pending': len(self.pending),
                        'django_visualization_drops': self.dropped}}
                encoded = (await asyncio.to_thread(json.dumps, payload, separators=(',', ':'))
                    if key[0] in FRAME_TYPES else json.dumps(payload, separators=(',', ':')))
                if self.current(payload):
                    await self.send(text_data=encoded)
                else:
                    self.dropped += 1
                    if payload.get('visualization_delivery_id') == self.inflight:
                        self.inflight = None
            self.wake.clear()

    async def close(self):
        self.task.cancel()
        try:
            await self.task
        except (asyncio.CancelledError, ConnectionError):
            pass
        self.pending.clear()
        self.applied.clear()
