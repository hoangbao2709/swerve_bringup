"""Opt-in callback timings; records labels and clocks only, never payloads."""
import logging
import os
import time
from functools import wraps

log = logging.getLogger(__name__)


def sample():
    return (time.monotonic(), time.thread_time())


def report(label, started):
    if os.environ.get('WARETWIN_MANUAL_TIMING') != '1':
        return
    wall = time.monotonic() - started[0]
    if wall >= .05:
        log.warning('CONTROL_CALLBACK_TIMING label=%s wall_ms=%.3f cpu_ms=%.3f',
                    label, wall * 1000, (time.thread_time() - started[1]) * 1000)


def profile_sync(function):
    @wraps(function)
    def timed(*args, **kwargs):
        if os.environ.get('WARETWIN_MANUAL_TIMING') != '1':
            return function(*args, **kwargs)
        started = sample()
        try:
            return function(*args, **kwargs)
        finally:
            report(function.__name__, started)
    return timed


def profile_async(function):
    @wraps(function)
    async def timed(*args, **kwargs):
        if os.environ.get('WARETWIN_MANUAL_TIMING') != '1':
            return await function(*args, **kwargs)
        started = sample()
        try:
            return await function(*args, **kwargs)
        finally:
            report(function.__qualname__, started)
    return timed


class DispatchTimingMixin:
    """Measure framework DB cleanup separately from the application handler."""
    async def dispatch(self, message):
        if os.environ.get('WARETWIN_MANUAL_TIMING') != '1':
            return await super().dispatch(message)
        from channels.consumer import get_handler_name
        from channels.db import aclose_old_connections
        handler = getattr(self, get_handler_name(message), None)
        if handler is None:
            return await super().dispatch(message)
        started = sample()
        await aclose_old_connections()
        report(type(self).__name__ + '.database_cleanup', started)
        await handler(message)
