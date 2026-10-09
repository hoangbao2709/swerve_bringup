#!/usr/bin/env python3
"""Serve installed SPA resources, with browser-host runtime backend settings."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import unquote, urlsplit


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, config=None, **kwargs):
        self.config = config or {}
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if urlsplit(self.path).path == '/runtime-config.js':
            body = ('window.WARETWIN_CONFIG=' + json.dumps(self.config) + ';').encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/javascript; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def end_headers(self):
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Permissions-Policy', 'camera=(), microphone=(), geolocation=()')
        self.send_header('Content-Security-Policy', "frame-ancestors 'none'; object-src 'none'; base-uri 'self'")
        path = urlsplit(self.path).path
        if path == '/index.html':
            self.send_header('Cache-Control', 'no-cache')
        elif path.startswith('/assets/') and Path(path).name:
            self.send_header('Cache-Control', 'public, max-age=31536000, immutable')
        elif path != '/runtime-config.js':
            self.send_header('Cache-Control', 'public, max-age=3600')
        super().end_headers()

    def send_head(self):
        relative = unquote(urlsplit(self.path).path).lstrip('/')
        root = Path(self.directory).resolve()
        target = (root / relative).resolve()
        if not target.is_relative_to(root):
            self.send_error(403)
            return None
        if not target.is_file():
            # Asset errors must remain errors. Only extensionless SPA routes
            # accept index.html; directory listing is never enabled.
            if Path(relative).suffix or relative.startswith(('api/', 'ws/', 'assets/')):
                self.send_error(404)
                return None
            self.path = '/index.html'
        return super().send_head()


class BoundedThreadingHTTPServer(ThreadingHTTPServer):
    """Bound concurrent request threads to limit trivial resource exhaustion."""
    daemon_threads = True
    block_on_close = False
    request_queue_size = 64

    def __init__(self, *args, max_clients=64, **kwargs):
        self._client_slots = threading.BoundedSemaphore(max_clients)
        super().__init__(*args, **kwargs)

    def process_request(self, request, client_address):
        if not self._client_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._client_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._client_slots.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', required=True, type=int)
    parser.add_argument('--backend-port', required=True)
    parser.add_argument('--api-url', default='')
    parser.add_argument('--ws-url', default='')
    parser.add_argument('--runtime-mode', default='GAZEBO_ROS')
    parser.add_argument('--demo-compact-view', choices=('true', 'false'), default='true')
    parser.add_argument('--same-origin-api', choices=('true', 'false'), default='false')
    args = parser.parse_args()
    if args.same_origin_api == 'true' and (args.api_url or args.ws_url):
        parser.error('same-origin API routing cannot be combined with explicit API/WebSocket URLs')
    config = dict(VITE_BACKEND_PORT=args.backend_port, VITE_API_BASE_URL=args.api_url,
                  VITE_API_URL='', VITE_WS_BASE_URL=args.ws_url, VITE_WS_URL='',
                  VITE_RUNTIME_MODE=args.runtime_mode,
                  VITE_DEMO_COMPACT_VIEW=args.demo_compact_view,
                  VITE_API_SAME_ORIGIN=args.same_origin_api)
    server = BoundedThreadingHTTPServer((args.host, args.port), partial(
        Handler, directory=args.directory, config=config))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
