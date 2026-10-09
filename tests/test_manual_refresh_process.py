"""Real child-process/IPC transport tests against a temporary unit WS server."""
import base64
import hashlib
import json
import socketserver
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from manual_refresh_process import ProcessManualRefreshWorker


def test_process_reuses_refresh_worker_and_stop_invalidates_old_holds():
    rows = []
    acquisitions = []
    class Handler(socketserver.BaseRequestHandler):
        def read(self, size):
            result = b''
            while len(result) < size:
                part = self.request.recv(size - len(result))
                if not part: raise EOFError()
                result += part
            return result
        def handle(self):
            header = b''
            while not header.endswith(b'\r\n\r\n'): header += self.read(1)
            key = next(line.split(b': ', 1)[1] for line in header.split(b'\r\n')
                if line.lower().startswith(b'sec-websocket-key:'))
            accept = base64.b64encode(hashlib.sha1(key + b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
            self.request.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ' + accept + b'\r\n\r\n')
            try:
                while True:
                    first, second = self.read(2)
                    size = second & 127
                    if size == 126: size = struct.unpack('!H', self.read(2))[0]
                    if size == 127: size = struct.unpack('!Q', self.read(8))[0]
                    mask = self.read(4) if second & 128 else None
                    payload = self.read(size)
                    if mask: payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
                    if first & 15 == 8:
                        self.request.sendall(b'\x88\x00'); return
                    if first & 15 == 1:
                        message = json.loads(payload)
                        if message['type'] == 'ROBOT_MANUAL': rows.append(message)
                        elif message['type'] == 'MANUAL_ACQUIRE': acquisitions.append(message)
            except (EOFError, ConnectionError): pass
    with socketserver.ThreadingTCPServer(('127.0.0.1', 0), Handler) as server:
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        sender = ProcessManualRefreshWorker(f'ws://127.0.0.1:{server.server_address[1]}/ws', 'R01', .02).start()
        try:
            sender.hold('FORWARD')
            end = time.monotonic() + 2
            while len(rows) < 4 and time.monotonic() < end: time.sleep(.01)
            assert len(rows) >= 4
            assert len(acquisitions) == 1
            assert acquisitions[0]['lease_id'] == rows[0]['lease_id']
            sender.stop()
            end = time.monotonic() + 2
            while not any(row['action'] == 'STOP' for row in rows) and time.monotonic() < end: time.sleep(.01)
            stopped = next(i for i, row in enumerate(rows) if row['action'] == 'STOP')
            time.sleep(.1)
            assert all(row['action'] == 'STOP' for row in rows[stopped:])
            assert sender.process.pid != __import__('os').getpid()
            assert sender.events and sender.events[-1]['action'] == 'STOP'
            assert not sender.error
        finally:
            sender.close(); server.shutdown(); thread.join(2)
        assert sender.process.returncode == 0 and not sender.reader.is_alive()
