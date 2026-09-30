"""Latest-only manual lane; safety barriers invalidate earlier motion."""
import queue
import threading
import time


class ControlMailbox:
    def __init__(self):
        self.lock = threading.RLock()
        self.generation = 0
        self.sequence = 0
        self.manual_sequence = 0
        self.manual = None
        self.controls = {}
        self.other = queue.Queue()

    def put(self, payload):
        kind = str(payload.get('type', '')).upper()
        with self.lock:
            self.sequence += 1
            data = dict(payload, _received_monotonic=time.monotonic(), _sequence=self.sequence)
            if kind in ('CONTROL_MODE', 'EMERGENCY_STOP', 'CLEAR_EMERGENCY_STOP', 'MANUAL_DISCONNECT') or (kind == 'MANUAL_CMD' and data.get('action') == 'STOP'):
                self.generation += 1
                self.manual_sequence = self.sequence
                self.manual = None
                data['_generation'] = self.generation
                self.controls[kind] = data
            elif kind == 'MANUAL_CMD':
                self.manual_sequence = self.sequence
                data['_generation'] = self.generation
                self.manual = data
            else:
                self.other.put(data)

    def get_nowait(self):
        with self.lock:
            if self.controls:
                kind = min(self.controls, key=lambda k: self.controls[k]['_sequence'])
                return self.controls.pop(kind)
            if self.manual is not None:
                data, self.manual = self.manual, None
                return data
            return self.other.get_nowait()

    def current(self, data):
        return (data.get('_generation') == self.generation
                and data.get('_sequence') == self.manual_sequence)
