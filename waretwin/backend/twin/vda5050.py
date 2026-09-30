"""Robot-scoped MQTT connection and VDA5050 task gate.

The broker adapter subscribes to order topics only when that robot's
``allow_task`` flag is enabled. An order that arrives during a configuration
transition is checked again before it is routed to the matching authenticated
ROS bridge.
"""
from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import logging
import re
import threading
import time
from typing import Any

from asgiref.sync import async_to_sync
from django.conf import settings
from cryptography.fernet import Fernet, InvalidToken

log = logging.getLogger(__name__)
_manager_lock = threading.RLock()
_clients: dict[str, Any] = {}
_states: dict[str, dict[str, Any]] = {}
_task_policy: dict[str, tuple[bool, bool]] = {}
_last_attempt: dict[str, float] = {}


def _cipher() -> Fernet:
    seed = str(getattr(settings, 'WARETWIN_VDA5050_ENCRYPTION_KEY', '') or settings.SECRET_KEY)
    key = base64.urlsafe_b64encode(hashlib.sha256(('waretwin-vda5050:' + seed).encode()).digest())
    return Fernet(key)


def encrypt_password(password: str) -> str:
    return _cipher().encrypt(password.encode('utf-8')).decode('ascii') if password else ''


def decrypt_password(value: str) -> str:
    if not value:
        return ''
    try:
        return _cipher().decrypt(value.encode('ascii')).decode('utf-8')
    except (InvalidToken, UnicodeError, ValueError) as exc:
        raise ValueError('stored MQTT password cannot be decrypted; configure it again') from exc


def public_configuration(config, connection_state: dict[str, Any] | None = None) -> dict[str, Any]:
    state = connection_state or get_connection_state(config.robot_id)
    return {
        'robot_id': config.robot_id,
        'enabled': config.enabled,
        'mqtt_host': config.mqtt_host,
        'mqtt_port': config.mqtt_port,
        'mqtt_username': config.mqtt_username,
        'password_configured': bool(config.mqtt_password_ciphertext),
        'tls_enabled': config.tls_enabled,
        'topic_prefix': config.topic_prefix,
        'interface_name': config.interface_name,
        'manufacturer': config.manufacturer,
        'serial_number': config.serial_number,
        'protocol_version': config.protocol_version,
        'mqtt_protocol_version': config.mqtt_protocol_version,
        'allow_task': config.allow_task,
        'allow_instant_actions': config.allow_instant_actions,
        'auto_reconnect': config.auto_reconnect,
        'reconnect_interval': config.reconnect_interval,
        'connection_timeout': config.connection_timeout,
        'keepalive': config.keepalive,
        'client_id': config.client_id,
        'connection_status': state.get('status', 'DISCONNECTED'),
        'last_error': state.get('last_error'),
        'ignored_orders': state.get('ignored_orders', 0),
        'updated_at': config.updated_at.isoformat() if config.updated_at else None,
    }


def validate_configuration(values: dict[str, Any]) -> dict[str, Any]:
    def text(key: str, default: str, maximum: int) -> str:
        value = str(values.get(key, default) or '').strip()
        if len(value) > maximum:
            raise ValueError(f'{key} is longer than {maximum} characters')
        return value

    def boolean(key: str, default: bool) -> bool:
        value = values.get(key, default)
        if not isinstance(value, bool):
            raise ValueError(f'{key} must be a boolean')
        return value

    host = text('mqtt_host', '', 255)
    enabled = boolean('enabled', False)
    if enabled and not host:
        raise ValueError('MQTT host is required when VDA5050 is enabled')
    if any(char.isspace() for char in host) or '/' in host:
        raise ValueError('MQTT host must be a hostname or IP address')
    try:
        port = int(values.get('mqtt_port', 1883))
    except (TypeError, ValueError) as exc:
        raise ValueError('MQTT port must be an integer') from exc
    if not 1 <= port <= 65535:
        raise ValueError('MQTT port must be between 1 and 65535')

    topic_prefix = text('topic_prefix', 'vda5050', 128).strip('/')
    topic_parts = topic_prefix.split('/')
    topic_segment_re = re.compile(r'^[A-Za-z0-9_.:-]+$')
    if not topic_prefix or any(not topic_segment_re.fullmatch(part) for part in topic_parts):
        raise ValueError('topic prefix contains an invalid MQTT topic segment')
    segment_re = topic_segment_re
    values_out = {
        'enabled': enabled, 'mqtt_host': host, 'mqtt_port': port,
        'mqtt_username': text('mqtt_username', '', 128),
        'tls_enabled': boolean('tls_enabled', False),
        'topic_prefix': topic_prefix,
        'interface_name': text('interface_name', 'uagv', 64),
        'manufacturer': text('manufacturer', 'PTAGV', 64),
        'serial_number': text('serial_number', '', 64),
        'protocol_version': text('protocol_version', '2.0.0', 16),
        'mqtt_protocol_version': text('mqtt_protocol_version', '3.1.1', 8),
        'allow_task': boolean('allow_task', True),
        'allow_instant_actions': boolean('allow_instant_actions', True),
        'auto_reconnect': boolean('auto_reconnect', True),
        'reconnect_interval': _bounded_int(values, 'reconnect_interval', 5, 1, 300),
        'connection_timeout': _bounded_int(values, 'connection_timeout', 5, 1, 60),
        'keepalive': _bounded_int(values, 'keepalive', 30, 5, 3600),
        'client_id': text('client_id', '', 128),
    }
    for key in ('interface_name', 'manufacturer', 'serial_number'):
        if values_out[key] and not segment_re.fullmatch(values_out[key]):
            raise ValueError(f'{key} contains characters that are invalid in an MQTT topic')
    if values_out['protocol_version'] not in ('2.0.0', '2.1.0', '3.0.0'):
        raise ValueError('supported VDA5050 versions are 2.0.0, 2.1.0, and 3.0.0')
    if values_out['mqtt_protocol_version'] not in ('3.1.1', '5.0'):
        raise ValueError('MQTT protocol must be 3.1.1 or 5.0')
    return values_out


def _bounded_int(values, key: str, default: int, low: int, high: int) -> int:
    try:
        value = int(values.get(key, default))
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{key} must be an integer') from exc
    if not low <= value <= high:
        raise ValueError(f'{key} must be between {low} and {high}')
    return value


def _mqtt_client(config, password: str, client_id: str | None = None):
    try:
        import paho.mqtt.client as mqtt
    except ImportError as exc:
        raise RuntimeError('paho-mqtt dependency is not installed') from exc
    protocol = mqtt.MQTTv5 if config.mqtt_protocol_version == '5.0' else mqtt.MQTTv311
    resolved_id = client_id or config.client_id or f'waretwin-{config.robot_id}'
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=resolved_id, protocol=protocol,
            reconnect_on_failure=bool(config.auto_reconnect),
        )
    except AttributeError:
        try:
            client = mqtt.Client(client_id=resolved_id, protocol=protocol,
                                 reconnect_on_failure=bool(config.auto_reconnect))
        except TypeError:
            # Compatibility for older Paho installations that predate the
            # reconnect_on_failure constructor option.
            client = mqtt.Client(client_id=resolved_id, protocol=protocol)
    if config.mqtt_username:
        client.username_pw_set(config.mqtt_username, password or None)
    if config.tls_enabled:
        client.tls_set()
    if config.auto_reconnect:
        client.reconnect_delay_set(min_delay=config.reconnect_interval,
                                   max_delay=max(config.reconnect_interval, 120))
    return client


def _topic_base(config) -> str:
    major = config.protocol_version.split('.', 1)[0]
    serial = config.serial_number or config.robot_id
    return '/'.join((config.topic_prefix.rstrip('/'), config.interface_name,
                     f'v{major}', config.manufacturer, serial))


def _on_order(config, client, message) -> None:
    suffix = message.topic.rsplit('/', 1)[-1]
    with _manager_lock:
        allow_task, allow_instant_actions = _task_policy.get(
            config.robot_id, (config.allow_task, config.allow_instant_actions))
    if suffix == 'order' and not allow_task:
        with _manager_lock:
            state = _states.setdefault(config.robot_id, {})
            state['ignored_orders'] = int(state.get('ignored_orders', 0)) + 1
        return
    if suffix == 'instantActions' and not allow_instant_actions:
        return
    if len(message.payload) > 256_000:
        log.warning('VDA5050 message rejected: payload too large for robot %s', config.robot_id)
        return
    try:
        body = json.loads(message.payload.decode('utf-8'))
        if not isinstance(body, dict):
            raise ValueError('message must be an object')
        from .ros_bridge_consumer import registry
        command_type = 'VDA5050_ORDER' if suffix == 'order' else 'VDA5050_INSTANT_ACTIONS'
        sent = async_to_sync(registry.send)({
            'type': command_type, 'robot_id': config.robot_id,
            'order' if suffix == 'order' else 'instant_actions': body,
        })
        if not sent:
            log.warning('VDA5050 %s not routed: R%s bridge is offline', suffix, config.robot_id)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError, RuntimeError) as exc:
        log.warning('VDA5050 %s rejected for robot %s: %s', suffix, config.robot_id, type(exc).__name__)


def _subscribe(client, config) -> None:
    root = _topic_base(config)
    topics = []
    if config.allow_task:
        topics.append((f'{root}/order', 0))
    if config.allow_instant_actions:
        topics.append((f'{root}/instantActions', 0))
    if topics:
        client.subscribe(topics)


def test_connection(config, password: str) -> dict[str, Any]:
    started = time.monotonic()
    connected = threading.Event()
    outcome: dict[str, Any] = {'ok': False, 'error_code': None, 'message': None}
    client = None
    try:
        client = _mqtt_client(config, password, f'waretwin-test-{config.robot_id}')

        def on_connect(_client, _userdata, _flags, reason_code, _properties=None):
            code = int(reason_code) if reason_code is not None else 1
            outcome['ok'] = code == 0
            if code != 0:
                outcome['error_code'] = 'MQTT_REFUSED'
                outcome['message'] = 'broker refused the connection'
            connected.set()

        client.on_connect = on_connect
        client.connect(config.mqtt_host, config.mqtt_port, keepalive=config.keepalive)
        client.loop_start()
        if not connected.wait(config.connection_timeout):
            outcome.update({'ok': False, 'error_code': 'TIMEOUT', 'message': 'broker connection timed out'})
    except Exception as exc:
        outcome.update({'ok': False, 'error_code': type(exc).__name__.upper(),
                        'message': 'broker connection failed'})
    finally:
        if client is not None:
            _stop_client(client, config.robot_id)
    outcome['latency_ms'] = round((time.monotonic() - started) * 1000.0, 1)
    outcome['broker'] = f'{config.mqtt_host}:{config.mqtt_port}'
    return outcome


def apply_configuration(config) -> dict[str, Any]:
    """Replace a live robot-scoped MQTT client; return real connection state."""
    robot_id = config.robot_id
    with _manager_lock:
        # Change policy before retiring the previous client so messages it
        # already queued are checked against the newly saved configuration.
        _task_policy[robot_id] = (bool(config.allow_task), bool(config.allow_instant_actions))
        _last_attempt[robot_id] = time.monotonic()
        old = _clients.pop(robot_id, None)
        _states[robot_id] = {'status': 'CONNECTING' if config.enabled else 'DISABLED',
                             'last_error': None, 'ignored_orders': _states.get(robot_id, {}).get('ignored_orders', 0)}
    if old is not None:
        _stop_client(old, robot_id)
    if not config.enabled:
        return get_connection_state(robot_id)

    connected = threading.Event()
    state = _states[robot_id]
    password = decrypt_password(config.mqtt_password_ciphertext)
    client = _mqtt_client(config, password)

    def on_connect(mqtt_client, _userdata, _flags, reason_code, _properties=None):
        code = int(reason_code) if reason_code is not None else 1
        if code == 0:
            _subscribe(mqtt_client, config)
            state.update({'status': 'CONNECTED', 'last_error': None})
        else:
            state.update({'status': 'ERROR', 'last_error': 'broker refused the connection'})
        connected.set()

    def on_disconnect(_client, _userdata, _disconnect_flags, reason_code, _properties=None):
        if int(reason_code or 0) != 0:
            state.update({
                'status': 'RECONNECTING' if config.auto_reconnect else 'DISCONNECTED',
                'last_error': 'broker connection lost',
            })

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = lambda c, _userdata, message: _on_order(config, c, message)
    client.connect(config.mqtt_host, config.mqtt_port, keepalive=config.keepalive)
    client.loop_start()
    with _manager_lock:
        _clients[robot_id] = client
    if not connected.wait(config.connection_timeout):
        state.update({'status': 'ERROR', 'last_error': 'broker connection timed out'})
        try:
            _stop_client(client, robot_id)
        finally:
            with _manager_lock:
                _clients.pop(robot_id, None)
        return get_connection_state(robot_id)
    if state['status'] != 'CONNECTED':
        with _manager_lock:
            _clients.pop(robot_id, None)
        try:
            _stop_client(client, robot_id)
        except Exception:
            pass
    return get_connection_state(robot_id)


def get_connection_state(robot_id: str) -> dict[str, Any]:
    with _manager_lock:
        return dict(_states.get(robot_id) or {'status': 'DISCONNECTED', 'last_error': None,
                                               'ignored_orders': 0})


def ensure_configuration_active(config) -> dict[str, Any]:
    """Restore a persisted enabled configuration when a robot console opens."""
    robot_id = config.robot_id
    if not config.enabled:
        return get_connection_state(robot_id)
    now = time.monotonic()
    with _manager_lock:
        state = _states.get(robot_id, {})
        client_exists = robot_id in _clients
        status = state.get('status')
        last_attempt = _last_attempt.get(robot_id, 0.0)
        if status in ('CONNECTED', 'CONNECTING', 'RECONNECTING') and client_exists:
            return dict(state)
        if now - last_attempt < max(1, int(config.reconnect_interval)):
            return dict(state or {'status': 'DISCONNECTED', 'last_error': None, 'ignored_orders': 0})
    try:
        return apply_configuration(config)
    except Exception as exc:
        with _manager_lock:
            failed = _states.setdefault(robot_id, {})
            failed.update({'status': 'ERROR', 'last_error': f'connection restore failed: {type(exc).__name__}'})
            return dict(failed)


def _stop_client(client, robot_id: str) -> None:
    # Paho loop_stop waits for its thread. Disconnect first so a healthy socket
    # does not keep that network-loop thread alive indefinitely.
    try:
        client.disconnect()
    except Exception:
        log.debug('VDA5050 MQTT disconnect failed for robot %s', robot_id)
    try:
        client.loop_stop()
    except Exception:
        log.warning('VDA5050 MQTT client shutdown failed for robot %s', robot_id)


def stop_all() -> None:
    with _manager_lock:
        clients = list(_clients.values())
        _clients.clear()
    for client in clients:
        _stop_client(client, 'shutdown')
