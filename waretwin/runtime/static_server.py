#!/usr/bin/env python3
"""Serve installed SPA resources, with browser-host runtime backend settings."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
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
    args = parser.parse_args()
    config = dict(VITE_BACKEND_PORT=args.backend_port, VITE_API_BASE_URL=args.api_url,
                  VITE_API_URL='', VITE_WS_BASE_URL=args.ws_url, VITE_WS_URL='',
                  VITE_RUNTIME_MODE=args.runtime_mode,
                  VITE_DEMO_COMPACT_VIEW=args.demo_compact_view)
    server = ThreadingHTTPServer((args.host, args.port), partial(
        Handler, directory=args.directory, config=config))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
