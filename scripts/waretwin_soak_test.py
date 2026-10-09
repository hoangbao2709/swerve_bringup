#!/usr/bin/env python3
"""Read-only WareTwin soak probe with periodic health/resource CSV export.

The probe sends no robot, navigation, mode, or E-Stop commands. For 24–72 hour
tests, point it at the HMI URLs and retain its CSV alongside the release logs.
WebSocket measurements use the read-only manual-channel handshake only.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time
from datetime import datetime, timezone
from urllib.request import urlopen
from urllib.parse import urlsplit


FIELDS = (
    'timestamp_utc', 'http_status', 'health_latency_ms', 'frontend_status',
    'frontend_latency_ms', 'backend_ready', 'database_ready', 'bridge_connected',
    'ros_connected', 'gazebo_ready', 'runtime_status', 'system_cpu_percent',
    'system_mem_available_bytes', 'owned_process_tree_rss_bytes',
    'owned_process_cpu_percent', 'websocket_status', 'websocket_latency_ms', 'error',
)


def http_probe(url: str, timeout: float) -> tuple[int | None, float | None, dict | None, str | None]:
    started = time.monotonic()
    try:
        with urlopen(url, timeout=timeout) as response:
            body = response.read(2_000_001)
            if len(body) > 2_000_000:
                return response.status, (time.monotonic() - started) * 1000, None, 'response-too-large'
            payload = json.loads(body) if 'json' in response.headers.get('Content-Type', '') else None
            return response.status, (time.monotonic() - started) * 1000, payload, None
    except Exception as exc:  # record each probe failure; do not hide it
        return None, (time.monotonic() - started) * 1000, None, type(exc).__name__


def _process_stat(pid: int) -> tuple[float, int] | None:
    """Return (CPU seconds, RSS bytes) for a process from Linux procfs."""
    try:
        raw = Path(f'/proc/{pid}/stat').read_text()
        fields = raw[raw.rfind(')') + 2:].split()  # fields start at kernel field 3
        cpu_ticks = int(fields[11]) + int(fields[12])  # utime + stime (fields 14 and 15)
        pages = int(Path(f'/proc/{pid}/statm').read_text().split()[1])
        return cpu_ticks / os.sysconf('SC_CLK_TCK'), pages * os.sysconf('SC_PAGE_SIZE')
    except (OSError, ValueError, IndexError):
        return None


def process_tree_metrics(root_pids: list[int]) -> tuple[float, int]:
    pids: set[int] = set()
    pending = list(root_pids)
    while pending:
        pid = pending.pop()
        if pid in pids or pid <= 0:
            continue
        pids.add(pid)
        children = Path(f'/proc/{pid}/task/{pid}/children')
        try:
            pending.extend(int(item) for item in children.read_text().split())
        except (OSError, ValueError):
            continue
    samples = [item for pid in pids if (item := _process_stat(pid)) is not None]
    return sum(item[0] for item in samples), sum(item[1] for item in samples)


def system_metrics() -> tuple[int | None, int | None]:
    try:
        first = Path('/proc/stat').read_text().splitlines()[0].split()[1:]
        values = [int(value) for value in first]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        mem = {}
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, value = line.split(':', 1)
            mem[key] = int(value.strip().split()[0]) * 1024
        return (total - idle), mem.get('MemAvailable')
    except (OSError, ValueError, IndexError):
        return None, None


def read_runtime_status(xdg_state_home: Path) -> tuple[str | None, list[int]]:
    registry = xdg_state_home / 'waretwin' / 'active-runtime.json'
    try:
        runtime_dir = Path(json.loads(registry.read_text())['runtime_dir'])
        status = json.loads((runtime_dir / 'web-status.json').read_text())
        pids = [int(status['pid'])]
        pids.extend(int(item['pid']) for item in status.get('children', []) if item.get('running'))
        return str(status.get('state') or status.get('status') or 'UNKNOWN'), pids
    except (OSError, ValueError, KeyError, TypeError):
        return None, []


def websocket_probe(url: str, origin: str, timeout: float) -> tuple[str, float | None]:
    started = time.monotonic()
    try:
        import websocket
        headers = []
        cookie = os.environ.get('WARETWIN_SOAK_WS_COOKIE', '').strip()
        if cookie:
            headers.append(f'Cookie: {cookie}')
        connection = websocket.create_connection(
            url, timeout=timeout, origin=origin, header=headers,
        )
        try:
            frame = connection.recv()
            payload = json.loads(frame) if isinstance(frame, str) else {}
            if payload.get('type') != 'ROBOT_MANUAL_CHANNEL_READY':
                return 'unexpected-handshake', (time.monotonic() - started) * 1000
        finally:
            connection.close()
        return 'ok', (time.monotonic() - started) * 1000
    except Exception as exc:
        return type(exc).__name__, (time.monotonic() - started) * 1000


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--health-url', default='http://127.0.0.1:8000/api/health/')
    parser.add_argument('--frontend-url', default='http://127.0.0.1:5173/')
    parser.add_argument('--websocket-url', default='')
    parser.add_argument('--origin', default='http://127.0.0.1:5173')
    parser.add_argument('--duration', type=float, required=True, help='duration in seconds; 86400 = 24 hours')
    parser.add_argument('--interval', type=float, default=30.0)
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--output', type=Path, required=True, help='new CSV path; existing evidence is never overwritten')
    parser.add_argument('--require-websocket', action='store_true')
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0 or args.timeout <= 0:
        parser.error('duration, interval, and timeout must be positive')
    out = args.output.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if out.exists():
        parser.error(f'refusing to overwrite existing soak evidence: {out}')
    ws_url = args.websocket_url
    if not ws_url:
        parsed = urlsplit(args.health_url)
        scheme = 'wss' if parsed.scheme == 'https' else 'ws'
        ws_url = f'{scheme}://{parsed.netloc}/ws/?control_only=1'
    start = time.monotonic()
    finish = start + args.duration
    prior_system = None
    prior_process_cpu = None
    prior_sample = None
    samples = 0
    failures = 0
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    fd = os.open(out, flags, 0o600)
    with os.fdopen(fd, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        while time.monotonic() <= finish:
            iteration_started = time.monotonic()
            status, health_ms, health, error = http_probe(args.health_url, args.timeout)
            frontend_status, frontend_ms, _frontend, frontend_error = http_probe(args.frontend_url, args.timeout)
            runtime_status, pids = read_runtime_status(
                Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser())
            cpu_total, mem_available = system_metrics()
            process_cpu, process_rss = process_tree_metrics(pids)
            elapsed = iteration_started - prior_sample if prior_sample is not None else None
            system_cpu = (100.0 * (cpu_total - prior_system) / max(
                1, elapsed * os.sysconf('SC_CLK_TCK') * (os.cpu_count() or 1))
                          if cpu_total is not None and prior_system is not None and elapsed else None)
            process_percent = (100.0 * (process_cpu - prior_process_cpu) / elapsed
                               if prior_process_cpu is not None and elapsed else None)
            ws_status, ws_ms = websocket_probe(ws_url, args.origin, args.timeout)
            row_error = ';'.join(item for item in (error, frontend_error) if item) or ''
            row = {
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'http_status': status, 'health_latency_ms': round(health_ms, 2) if health_ms is not None else '',
                'frontend_status': frontend_status,
                'frontend_latency_ms': round(frontend_ms, 2) if frontend_ms is not None else '',
                'backend_ready': health.get('backend_ready', '') if health else '',
                'database_ready': health.get('database', '') if health else '',
                'bridge_connected': health.get('ros_bridge', '') if health else '',
                'ros_connected': health.get('ros', '') if health else '',
                'gazebo_ready': health.get('gazebo', '') if health else '',
                'runtime_status': runtime_status or '',
                'system_cpu_percent': round(system_cpu, 2) if system_cpu is not None else '',
                'system_mem_available_bytes': mem_available if mem_available is not None else '',
                'owned_process_tree_rss_bytes': process_rss,
                'owned_process_cpu_percent': round(process_percent, 2) if process_percent is not None else '',
                'websocket_status': ws_status,
                'websocket_latency_ms': round(ws_ms, 2) if ws_ms is not None else '',
                'error': row_error,
            }
            writer.writerow(row)
            stream.flush()
            samples += 1
            if status != 200 or frontend_status != 200 or (health and health.get('database') is not True):
                failures += 1
            if args.require_websocket and ws_status != 'ok':
                failures += 1
            prior_system, prior_process_cpu, prior_sample = cpu_total, process_cpu, iteration_started
            remaining = args.interval - (time.monotonic() - iteration_started)
            if remaining > 0 and time.monotonic() < finish:
                time.sleep(min(remaining, finish - time.monotonic()))
    print(f'samples={samples} failed_samples={failures} duration_s={time.monotonic() - start:.1f} csv={out.resolve()}')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
