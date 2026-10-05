"""Bounded priority status and latest-only telemetry, with one socket writer."""
import threading
from collections import deque


class OutboundMailbox:
    LATEST = frozenset({
        '_SOCKET_PING',
        'ROBOT_DETAIL_VIEW_STATUS',
        'HEARTBEAT', 'ROBOT_STATE', 'ROS_DIAGNOSTICS', 'SYSTEM_DIAGNOSTICS',
        'COMMAND_DIAGNOSTICS', 'LIDAR_SCAN', 'LIDAR_MAP_2D', 'LIDAR_MAP_3D',
        'LIDAR_STREAM_DIAGNOSTICS', 'MAP_SNAPSHOT', 'NAV_GLOBAL_PATH',
        'NAV_LOCAL_PATH', 'MAP_REVISION_STATUS', 'NAVIGATION_MAP_STATUS',
    })

    def __init__(self, capacity=64):
        self.capacity = capacity
        self.condition = threading.Condition()
        self.critical = deque()
        self.latest = {}
        self.epoch = 0
        self.dropped = 0
        self.dropped_by_type = {}

    def clear(self):
        with self.condition:
            self.epoch += 1
            self.critical.clear()
            self.latest.clear()
            self.condition.notify_all()

    def discard_views(self):
        with self.condition:
            for kind in ('LIDAR_SCAN', 'LIDAR_MAP_2D', 'LIDAR_MAP_3D', 'LIDAR_STREAM_DIAGNOSTICS'):
                self.latest.pop(kind, None)

    def offer(self, payload):
        with self.condition:
            item = (self.epoch, payload)
            kind = payload.get('type')
            if kind in self.LATEST:
                if kind in self.latest:
                    self.dropped += 1
                    self.dropped_by_type[kind] = self.dropped_by_type.get(kind, 0) + 1
                self.latest[kind] = item
            else:
                if len(self.critical) >= self.capacity:
                    raise BufferError('critical bridge output capacity exceeded')
                self.critical.append(item)
            self.condition.notify()

    def take(self, timeout=0.25):
        with self.condition:
            self.condition.wait_for(lambda: self.critical or self.latest, timeout)
            if self.critical:
                return self.critical.popleft()
            if self.latest:
                kind = ('ROBOT_DETAIL_VIEW_STATUS' if 'ROBOT_DETAIL_VIEW_STATUS' in self.latest
                    else next(iter(self.latest)))
                return self.latest.pop(kind)
            return None

    def current(self, epoch):
        with self.condition:
            return epoch == self.epoch
