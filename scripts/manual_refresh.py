"""Dedicated bounded Web manual sender, independent of ROS/telemetry probing."""
import threading
import time
import uuid
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
        self.lease_id = None
        self.lease_acquisition_sent = False
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
            if self.action is None:
                self.lease_id = uuid.uuid4().hex
                self.lease_acquisition_sent = False
            self.generation += 1
            self.action = action
        self.wake.set()

    def _emit(self, action, generation=None, lease_id=None):
        with self.write_lock:
            with self.state_lock:
                if generation is not None and (generation != self.generation or action != self.action):
                    return
                self.sequence += 1
                sequence = self.sequence
                command_lease_id = lease_id if lease_id is not None else self.lease_id
                acquire = action != 'STOP' and not self.lease_acquisition_sent
                if acquire:
                    self.lease_acquisition_sent = True
            if acquire:
                self.send({'type': 'MANUAL_ACQUIRE', 'robot_id': self.robot_id,
                           'lease_id': command_lease_id})
            started = time.monotonic()
            payload = {'type': 'ROBOT_MANUAL', 'robot_id': self.robot_id,
                       'action': action, 'sequence_id': sequence, 'client_monotonic': started}
            if command_lease_id is not None:
                payload['lease_id'] = command_lease_id
            self.send(payload)
            self.events.append({'sequence_id': sequence, 'action': action,
                                'T0': started, 'send_completed': time.monotonic()})

    def stop(self):
        # Invalidate immediately, even if a previous network write is finishing.
        with self.state_lock:
            self.generation += 1
            self.action = None
            lease_id, self.lease_id = self.lease_id, None
            self.lease_acquisition_sent = False
        self.wake.set()
        self._emit('STOP', lease_id=lease_id)

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
            started = time.monotonic()
            if action is not None:
                try:
                    self._emit(action, generation)
                except Exception as exc:
                    self.error = exc
                    with self.state_lock:
                        self.action = None
                        self.lease_id = None
                        self.lease_acquisition_sent = False
            # Pace starts, not completions: network duration must not be added
            # to every refresh period. No overdue ticks/commands are queued.
            self.wake.wait(max(0, started + self.interval - time.monotonic()))
