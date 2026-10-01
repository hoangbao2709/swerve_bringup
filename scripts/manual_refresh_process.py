"""Isolate the existing refresh worker from the diagnostic ROS probe's GIL.

Only acceptance uses this adapter. Commands still use an authenticated Django
WebSocket. One synchronous IPC command is in flight; there is no command FIFO.
Secrets travel through stdin, never process arguments or diagnostic output.
"""
import json
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path


class ProcessManualRefreshWorker:
    def __init__(self, url, robot_id, interval=.1, on_message=None):
        self.url, self.robot_id, self.interval = url, robot_id, interval
        self.on_message = on_message
        self.events = deque(maxlen=2048)
        self.error = None
        self.condition = threading.Condition()
        self.command_lock = threading.Lock()
        self.ack = -1
        self.command_id = 0
        self.process = None
        self.closing = False

    def start(self):
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, name='manual-process-receipts', daemon=True)
        self.reader.start()
        try:
            self._request({'url': self.url, 'robot_id': self.robot_id, 'interval': self.interval})
        except Exception:
            self.closing = True
            self.process.terminate(); self.process.wait(timeout=3)
            self.reader.join(3)
            self.process.stdin.close(); self.process.stdout.close()
            raise
        return self

    def _read(self):
        for line in self.process.stdout:
            message = json.loads(line)
            with self.condition:
                if message['kind'] == 'ack': self.ack = message['id']
                elif message['kind'] == 'error': self.error = RuntimeError(message['error'])
                else:
                    self.events.append(message['event'])
                    if self.on_message: self.on_message(message['message'])
                self.condition.notify_all()
        with self.condition:
            if not self.closing:
                self.error = self.error or RuntimeError('manual refresh process disconnected')
            self.condition.notify_all()

    def _request(self, payload):
        with self.command_lock:
            ident = self.command_id; self.command_id += 1
            self.process.stdin.write(json.dumps({'id': ident, **payload}) + '\n')
            self.process.stdin.flush()
            with self.condition:
                if not self.condition.wait_for(lambda: self.ack == ident or self.error, timeout=3):
                    raise RuntimeError('manual refresh IPC acknowledgement timeout')
                if self.error: raise self.error

    def hold(self, action):
        self._request({'command': 'hold', 'action': action})

    def stop(self):
        self._request({'command': 'stop'})

    def close(self):
        if self.process is None: return
        self.closing = True
        try:
            self._request({'command': 'close'})
        finally:
            self.process.stdin.close()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate(); self.process.wait(timeout=3)
            self.reader.join(3)
            self.process.stdout.close()


def worker():
    import websocket
    from manual_refresh import ManualRefreshWorker
    output_lock = threading.Lock()
    def output(payload):
        with output_lock:
            print(json.dumps(payload), flush=True)
    init = json.loads(sys.stdin.readline())
    ws = websocket.create_connection(init['url'], timeout=1)
    # This non-rendering control client uses bounded display delivery rather
    # than receiving/decoding a duplicate LiDAR stream beside the real browser.
    # With no frame receipt, one disposable frame remains in flight; control
    # status and commands remain independent of visualization backpressure.
    ws.send(json.dumps({'type': 'ROBOT_DETAIL_VIEW', 'robot_id': init['robot_id'],
        'view': 'GLOBAL', 'request_id': 'manual-process-initial', 'delivery_ack': True}))
    closed = threading.Event()
    def drain():
        # Do not deserialize visualization JSON in the control sender.
        while not closed.is_set():
            try: ws.recv()
            except websocket.WebSocketTimeoutException: continue
            except Exception:
                if not closed.is_set(): output({'kind': 'error', 'error': 'manual WebSocket disconnected'})
                return
    reader = threading.Thread(target=drain, daemon=True); reader.start()
    def send(message):
        try: ws.send(json.dumps(message))
        except Exception as exc:
            output({'kind': 'error', 'error': type(exc).__name__})
            raise
        output({'kind': 'event', 'message': message, 'event': {
            'sequence_id': message['sequence_id'], 'action': message['action'],
            'T0': message['client_monotonic'], 'send_completed': time.monotonic()}})
    sender = ManualRefreshWorker(send, init['robot_id'], init['interval']).start()
    output({'kind': 'ack', 'id': init['id']})
    try:
        for line in sys.stdin:
            request = json.loads(line)
            if request['command'] == 'hold': sender.hold(request['action'])
            elif request['command'] == 'stop': sender.stop()
            elif request['command'] == 'close':
                sender.close(); output({'kind': 'ack', 'id': request['id']}); break
            else: raise ValueError('invalid refresh command')
            output({'kind': 'ack', 'id': request['id']})
    finally:
        closed.set()
        try: sender.close()
        finally: ws.close(); reader.join(2)


if __name__ == '__main__':
    try: worker()
    except Exception as exc:
        print(json.dumps({'kind': 'error', 'error': type(exc).__name__}), flush=True)
        sys.exit(1)
