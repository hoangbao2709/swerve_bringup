#!/usr/bin/env python3
"""Ownership-checked status/stop for an installed launch (Linux)."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
from urllib.request import urlopen
from service import default_runtime, state_home


def running(root):
    try:
        status = json.loads((root / 'web-status.json').read_text())
        pid = int(status['pid'])
        command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        started = Path(f'/proc/{pid}/stat').read_text().split(')')[1].split()[19]
        if (status['state'] == 'STOPPED' or started != status['process_start']
                or not any(arg.endswith(b'/runtime/service.py') for arg in command)
                or Path(status['runtime_dir']).resolve() != root):
            return None
        return status
    except (OSError, ValueError, KeyError):
        return None


def managed(root):
    """Whether this runtime directory has been claimed by an installed launch."""
    try:
        status = json.loads((root / 'web-status.json').read_text())
        return Path(status['runtime_dir']).resolve() == root
    except (OSError, ValueError, KeyError):
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-dir', default='')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--active', action='store_true')
    parser.add_argument('--managed', action='store_true')
    parser.add_argument('--backend-port', default='')
    parser.add_argument('--frontend-port', default='')
    args = parser.parse_args()
    root = Path(args.runtime_dir).expanduser().resolve() if args.runtime_dir else None
    if root is None:
        registry = state_home() / 'waretwin/active-runtime.json'
        try:
            root = Path(json.loads(registry.read_text())['runtime_dir']).resolve()
        except (OSError, ValueError, KeyError):
            root = default_runtime()
    status = running(root)
    if args.managed:
        return 0 if status or managed(root) else 1
    if not status:
        if not args.active:
            print('WareTwin installed runtime is not running')
        if args.stop:
            return 0
        return 1
    if args.active:
        return 0
    if args.backend_port:
        status['backend_url'] = status['backend_url'].rsplit(':', 1)[0] + ':' + args.backend_port
    if args.frontend_port:
        status['frontend_url'] = status['frontend_url'].rsplit(':', 1)[0] + ':' + args.frontend_port
    if args.stop:
        os.kill(status['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and running(root):
            time.sleep(.2)
        if running(root):
            print('Owned runtime did not stop within 60 seconds; inspect its logs')
            return 1
        print('WareTwin installed runtime stopped')
        return 0
    print(json.dumps(status, indent=2))
    try:
        for url in (status['backend_url'] + '/api/health/', status['frontend_url']):
            with urlopen(url, timeout=3) as response:
                if response.status != 200:
                    return 1
    except OSError:
        return 1
    return 0 if status['state'] in ('WEB_READY', 'READY') else 1


if __name__ == '__main__':
    raise SystemExit(main())
