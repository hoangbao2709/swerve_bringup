"""Dedicated bounded Web manual sender, independent of ROS/telemetry probing."""
import threading
import time
from collections import deque


class ManualRefreshWorker:
    def __init__(self, send, robot_id, interval=0.1):
        if interval <= 0:
            raise ValueError('interval must be positive')
        self.send, self.robot_id, self.interval = send, robot_id, interval
        self.state_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.wake = threading.Event()
        self.shutdown = threading.Event()
        self.action = None
        self.generation = self.sequence = 0
        self.events = deque(maxlen=2048)
        self.error = None
        self.thread = threading.Thread(target=self._run, name='manual-refresh', daemon=True)

    def start(self):
        self.thread.start()
        return self

    def hold(self, action):
        if action == 'STOP':
            return self.stop()
        with self.state_lock:
            self.generation += 1
            self.action = action
        self.wake.set()

    def _emit(self, action, generation=None):
        with self.write_lock:
            with self.state_lock:
                if generation is not None and (generation != self.generation or action != self.action):
                    return
                self.sequence += 1
                sequence = self.sequence
            started = time.monotonic()
            self.send({'type': 'ROBOT_MANUAL', 'robot_id': self.robot_id,
                       'action': action, 'sequence_id': sequence, 'client_monotonic': started})
            self.events.append({'sequence_id': sequence, 'action': action,
                                'T0': started, 'send_completed': time.monotonic()})

    def stop(self):
        # Invalidate immediately, even if a previous network write is finishing.
        with self.state_lock:
            self.generation += 1
            self.action = None
        self.wake.set()
        self._emit('STOP')

    def close(self):
        try:
            self.stop()
        finally:
            self.shutdown.set()
            self.wake.set()
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                raise RuntimeError('manual sender did not stop; transport must use a bounded timeout')

    def _run(self):
        while not self.shutdown.is_set():
            self.wake.clear()
            with self.state_lock:
                action, generation = self.action, self.generation
            if action is not None:
                try:
                    self._emit(action, generation)
                except Exception as exc:
                    self.error = exc
                    with self.state_lock:
                        self.action = None
            self.wake.wait(self.interval)
