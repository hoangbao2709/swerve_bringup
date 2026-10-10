"""Fail-closed local-HMI and trusted-gateway authorization boundary.

The browser does not hold a robot bearer token. In PROTECTED_LAN mode, an
authenticated same-host gateway must strip client-supplied X-WareTwin-*
headers and sign a short-lived assertion for each HTTP request and WebSocket
handshake. The assertion binds identity, permissions, robot scope, method,
path, Origin, and expiry.
"""
from __future__ import annotations

import hashlib
import hmac
import ipaddress
import logging
import re
import time
from copy import deepcopy
from urllib.parse import urlsplit

from django.conf import settings

log = logging.getLogger('twin.operator_security')

SAFE_METHODS = frozenset({'GET', 'HEAD', 'OPTIONS'})
ASSERTION_HEADERS = {
    'identity': b'x-waretwin-identity',
    'permissions': b'x-waretwin-permissions',
    'robots': b'x-waretwin-robots',
    'expires': b'x-waretwin-expires',
    'signature': b'x-waretwin-signature',
}
TOKEN_RE = re.compile(r'^[A-Za-z0-9:_.*-]{1,96}$')
IDENTITY_RE = re.compile(r'^[A-Za-z0-9@._:+-]{1,128}$')
ROBOT_RE = re.compile(r'^[A-Za-z0-9._:-]{1,64}$')
MAX_ASSERTION_LIFETIME_S = 300


def mode() -> str:
    return str(getattr(settings, 'WARETWIN_OPERATOR_AUTH_MODE', 'LOCAL_LOOPBACK')).upper()


def _header_values(scope: dict) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for raw_name, raw_value in scope.get('headers', ()):
        name = raw_name.decode('latin1').lower()
        try:
            value = raw_value.decode('ascii')
        except UnicodeDecodeError:
            value = ''
        result.setdefault(name, []).append(value)
    return result


def request_origin(scope: dict) -> str:
    values = _header_values(scope).get('origin', [])
    return values[0] if len(values) == 1 else ''


def origin_allowed(origin: str, *, required: bool = False) -> bool:
    if not origin:
        return not required
    if origin == 'null' or origin not in set(getattr(settings, 'WARETWIN_OPERATOR_ALLOWED_ORIGINS', ())):
        return False
    return True


def _peer_ip(scope: dict):
    client = scope.get('client')
    if not client or not client[0]:
        return None
    try:
        return ipaddress.ip_address(str(client[0]).split('%', 1)[0])
    except ValueError:
        return None


def peer_is_loopback(scope: dict) -> bool:
    address = _peer_ip(scope)
    return bool(address and address.is_loopback)


def peer_is_trusted_proxy(scope: dict) -> bool:
    address = _peer_ip(scope)
    if address is None:
        return False
    return any(address in network for network in getattr(settings, 'WARETWIN_TRUSTED_PROXY_NETWORKS', ()))


def _canonical(method: str, path: str, origin: str, identity: str,
               permissions: str, robots: str, expires: str) -> bytes:
    fields = ('waretwin-operator-v1', method.upper(), path, origin,
              identity, permissions, robots, expires)
    return '\n'.join(fields).encode('utf-8')


def sign_assertion(*, method: str, path: str, origin: str, identity: str,
                   permissions: str, robots: str, expires: str, secret: str) -> str:
    """Gateway/test helper; the secret must remain server-side."""
    return hmac.new(secret.encode('utf-8'), _canonical(
        method, path, origin, identity, permissions, robots, expires), hashlib.sha256).hexdigest()


def verify_assertion(scope: dict) -> dict | None:
    headers = _header_values(scope)
    values = {}
    for key, name in ASSERTION_HEADERS.items():
        found = headers.get(name.decode('ascii'), [])
        if len(found) != 1 or not found[0]:
            return None
        values[key] = found[0]
    identity = values['identity']
    permission_values = values['permissions'].split(',')
    robot_values = values['robots'].split(',')
    if not IDENTITY_RE.fullmatch(identity):
        return None
    if (not permission_values or any(not TOKEN_RE.fullmatch(item) for item in permission_values)
            or len(permission_values) != len(set(permission_values))):
        return None
    if (not robot_values or any(item != '*' and not ROBOT_RE.fullmatch(item) for item in robot_values)
            or len(robot_values) != len(set(robot_values))):
        return None
    try:
        expires = int(values['expires'])
    except ValueError:
        return None
    now = int(time.time())
    if expires <= now or expires > now + MAX_ASSERTION_LIFETIME_S:
        return None
    origin = request_origin(scope)
    expected = sign_assertion(method=scope.get('method', 'GET'), path=scope.get('path', ''),
                              origin=origin, identity=identity,
                              permissions=values['permissions'], robots=values['robots'],
                              expires=values['expires'],
                              secret=str(getattr(settings, 'WARETWIN_OPERATOR_GATEWAY_SECRET', '') or ''))
    if not hmac.compare_digest(expected, values['signature'].lower()):
        return None
    return {
        'identity': identity,
        'permissions': frozenset(permission_values),
        'robots': frozenset(robot_values),
        'expires_at': float(expires),
    }


def _robot_id(path: str) -> str | None:
    parts = [part for part in path.split('/') if part]
    if len(parts) >= 3 and parts[0] == 'api' and parts[1] == 'robots':
        return parts[2]
    return None


def http_permission(path: str, method: str) -> tuple[str | None, str | None]:
    """Map every unsafe API route to a capability; unknown writes fail closed."""
    robot_id = _robot_id(path)
    if path.rstrip('/') == '/api/health' and method in SAFE_METHODS:
        return None, None
    if method in SAFE_METHODS:
        return 'operator:read', robot_id

    normalized = path.rstrip('/')
    if normalized.endswith('/emergency-stop'):
        return 'safety:estop', robot_id
    if normalized.endswith('/clear-emergency-stop'):
        return 'safety:reset', robot_id
    if normalized.startswith('/api/robots/') and '/local/' in normalized:
        if normalized.endswith('/initial-pose'):
            return 'localization:write', robot_id
        if '/vda5050' in normalized:
            return 'vda5050:write', robot_id
        if normalized.endswith('/runtime-mode'):
            return 'runtime:mode', robot_id
        if any(token in normalized for token in ('/mapping/', '/maps/save', '/maps/load', '/maps/resume-session')):
            return 'map:write', robot_id
    if normalized.endswith('/map-registration'):
        return 'map:write', robot_id
    if normalized == '/api/navigation/missions/start':
        return 'navigation:goal', None
    mission = re.fullmatch(r'/api/navigation/missions/\d+/(pause|resume|cancel|replan)', normalized)
    if mission:
        return {
            'pause': 'navigation:pause', 'resume': 'navigation:resume',
            'cancel': 'navigation:cancel', 'replan': 'navigation:goal',
        }[mission.group(1)], None
    if normalized.startswith('/api/navigation/'):
        return 'navigation:goal', None
    if normalized.startswith('/api/conveyors/'):
        return 'robot:control', None
    if normalized.startswith('/api/scheduler/') or normalized.startswith('/api/orders') \
            or normalized.startswith('/api/schedules') or normalized.startswith('/api/tasks'):
        return 'fleet:dispatch', None
    if normalized.startswith('/api/warehouse') or normalized.startswith('/api/warehouses') \
            or normalized.startswith('/api/layout/'):
        return 'warehouse:write', None
    return 'operator:admin', robot_id


def authorized(operator: dict | None, permission: str, robot_id: str | None = None) -> bool:
    if mode() == 'LOCAL_LOOPBACK':
        return True
    if not has_permission(operator, permission):
        return False
    robots = operator.get('robots', frozenset())
    if robot_id:
        return '*' in robots or robot_id in robots
    return '*' in robots


def has_permission(operator: dict | None, permission: str) -> bool:
    """Check identity/capability without implying fleet-wide robot access.

    WebSocket connection establishment and RESYNC are operator-level actions:
    robot scope is enforced on each command and every outbound payload instead.
    """
    if mode() == 'LOCAL_LOOPBACK':
        return True
    if not operator or float(operator.get('expires_at', 0)) <= time.time():
        return False
    permissions = operator.get('permissions', frozenset())
    return 'operator:admin' in permissions or permission in permissions


def _robot_ids(operator: dict) -> set[str] | None:
    robots = operator.get('robots', frozenset())
    if '*' in robots:
        return None
    return {str(robot_id) for robot_id in robots}


def _filter_robot_mapping(value, allowed: set[str]):
    if not isinstance(value, dict):
        return {}
    return {key: row for key, row in value.items()
            if str(key) in allowed and isinstance(row, dict)}


def _scoped_twin_state(state: dict, allowed: set[str]) -> dict:
    """Keep robot-owned state and omit environment data with fleet attribution."""
    if not isinstance(state, dict):
        return {}
    robots = state.get('robots') if isinstance(state.get('robots'), dict) else {}
    visible_robots = {rid: deepcopy(row) for rid, row in robots.items()
                      if str(rid) in allowed and isinstance(row, dict)}
    def owned_rows(section):
        entries = state.get(section)
        return ({key: deepcopy(value) for key, value in entries.items()
                 if isinstance(value, dict)
                 and str(value.get('robot_id') or '') in allowed}
                if isinstance(entries, dict) else {})

    entries = state.get('recent_events')
    visible_events = ([deepcopy(value) for value in entries if isinstance(value, dict)
                       and str(value.get('robot_id') or '') in allowed]
                      if isinstance(entries, list) else [])
    original_sim = state.get('sim') if isinstance(state.get('sim'), dict) else {}
    sim = {key: original_sim[key] for key in ('tick', 'tick_ms') if key in original_sim}
    # These sections contain cross-robot aggregates or ownership fields (for
    # example zone robot_count and lift occupant/queues). Their records do not
    # have a sufficiently reliable robot-scope contract, so scoped operators
    # receive empty collections. Wildcard administrators retain the full UI.
    result = {
        'schema_version': state.get('schema_version', '1.0'),
        'layout_id': state.get('layout_id'),
        'sim': sim,
        'robots': visible_robots,
        'tasks': owned_rows('tasks'),
        'alerts': owned_rows('alerts'),
        'recent_events': visible_events,
        'recent_decisions': [],
        'lifts': {}, 'zones': {}, 'conveyors': {}, 'cameras': {},
        'sensors': {}, 'people': {}, 'subsystems': {},
    }
    kpi = state.get('kpi') if isinstance(state.get('kpi'), dict) else {}
    statuses = [str(row.get('status') or '').upper() for row in visible_robots.values()]
    result['kpi'] = {
        'tick': int(kpi.get('tick') or 0),
        'fleet': {
            'total': len(visible_robots),
            'active': statuses.count('ACTIVE'),
            'charging': statuses.count('CHARGING'),
            'idle': statuses.count('IDLE'),
            'warning': statuses.count('WARNING'),
            'error': statuses.count('ERROR'),
            'offline': statuses.count('OFFLINE'),
        },
        'operation': {'throughput_per_min': 0, 'completed_today': 0,
                      'completed_target': 0, 'pending': 0, 'ongoing': 0,
                      'avg_task_time_s': 0, 'on_time_rate': 0, 'avg_utilization': 0},
        'efficiency': {'avg_travel_distance_m': 0, 'avg_wait_time_s': 0,
                       'congestion_index': 0, 'energy_kwh': 0},
        'throughput_series': [],
        'lifts': {'trips': 0, 'utilization': 0, 'avg_wait_s': 0, 'faults': 0},
    }
    return result


def scope_operator_payload(payload: dict, operator: dict | None) -> dict | None:
    """Return only data authorized for a protected robot-scoped WebSocket.

    A payload with ambiguous/global robot ownership is withheld by default.
    The unrestricted loopback and wildcard policies retain the legacy wire
    contract. This function operates on copies so the shared runtime state is
    never modified for another client.
    """
    if not isinstance(payload, dict):
        return None
    if mode() == 'LOCAL_LOOPBACK':
        return payload
    if not operator or float(operator.get('expires_at', 0)) <= time.time():
        return None
    allowed = _robot_ids(operator)
    if allowed is None:
        return payload
    kind = str(payload.get('type') or '').upper()

    if kind == 'FULL':
        result = deepcopy(payload)
        result['state'] = _scoped_twin_state(result.get('state'), allowed)
        return result
    if kind == 'PATCH':
        patch = payload.get('patch') if isinstance(payload.get('patch'), dict) else {}
        scoped = {}
        for section, value in patch.items():
            if section == 'robots':
                scoped[section] = _filter_robot_mapping(value, allowed)
            elif section in ('tasks', 'alerts') and isinstance(value, dict):
                # A deletion tombstone has no robot owner. Withhold it rather
                # than revealing another robot's task/alert identifier.
                scoped[section] = {key: deepcopy(row) for key, row in value.items()
                                   if isinstance(row, dict)
                                   and str(row.get('robot_id') or '') in allowed}
            elif section == 'sim' and isinstance(value, dict):
                scoped[section] = {key: value[key] for key in ('tick', 'tick_ms')
                                   if key in value}
            # Zones, lifts, conveyors, cameras, sensors, people, KPI, and
            # unknown sections are withheld because they can encode fleet state.
        events = payload.get('events')
        filtered_events = ([deepcopy(row) for row in events if isinstance(row, dict)
                            and str(row.get('robot_id') or '') in allowed]
                           if isinstance(events, list) else [])
        return {
            'type': 'PATCH', 'base_tick': payload.get('base_tick'),
            'tick': payload.get('tick'), 'patch': scoped, 'events': filtered_events,
        }
    if kind == 'RUNTIME_STATUS':
        connected = sorted(set(payload.get('connected_robot_ids') or ()) & allowed)
        maps = _filter_robot_mapping(payload.get('local_active_maps'), allowed)
        sync = _filter_robot_mapping(payload.get('robot_map_sync'), allowed)
        navigation = _filter_robot_mapping(payload.get('navigation_maps'), allowed)
        capabilities = _filter_robot_mapping(payload.get('robot_capabilities'), allowed)
        sessions = payload.get('robot_mapping_sessions')
        sessions = ({rid: value for rid, value in sessions.items() if str(rid) in allowed}
                    if isinstance(sessions, dict) else {})
        selected = next(iter(connected or sorted(allowed)), None)
        row = sync.get(selected, {}) if selected else {}
        active = maps.get(selected, {}) if selected else {}
        cap = capabilities.get(selected, {}) if selected else {}
        result = {
            'type': 'RUNTIME_STATUS',
            'runtime_mode': payload.get('runtime_mode'),
            'runtime_state': payload.get('runtime_state'),
            'bridge_state': 'CONNECTED' if bool(connected) else 'DISCONNECTED',
            'ros_connected': bool(connected),
            'connected_robot_ids': connected,
            'nav2_state': ('READY' if cap.get('nav2_ready') else
                           'CONNECTED' if connected else 'OFFLINE'),
            'last_telemetry_at': None,
            'published_revision': active.get('canonical_map_revision'),
            'published_version': payload.get('published_version'),
            'ros_revision': row.get('ros_revision'),
            'gazebo_revision': row.get('gazebo_revision'),
            'nav2_revision': row.get('nav2_revision'),
            'tag_map_revision': row.get('tag_map_revision'),
            'tf_status': row.get('tf_status', False),
            'map_sync_status': active.get('map_sync_status', 'PENDING'),
            'map_sync_error': row.get('error'),
            'robot_map_sync': sync,
            'navigation_maps': navigation,
            'local_active_maps': maps,
            'robot_capabilities': capabilities,
            'robot_mapping_sessions': sessions,
        }
        return result

    robot_id = str(payload.get('robot_id') or '')
    if not robot_id:
        for nested in ('map', 'scan', 'controller', 'diagnostics'):
            value = payload.get(nested)
            if isinstance(value, dict) and value.get('robot_id'):
                robot_id = str(value['robot_id'])
                break
    if robot_id:
        return payload if robot_id in allowed else None
    if kind == 'ERROR':
        safe_errors = {
            'OPERATOR_PERMISSION_REQUIRED': 'operator is not authorized for this operation',
            'BAD_MESSAGE': 'message format is invalid',
            'RATE_LIMITED': 'request rate limit exceeded',
            'MANUAL_RATE_LIMITED': 'manual command rate exceeded; release control and retry',
            'CONTROL_CHANNEL_RESTRICTED': 'operation is unavailable on this control channel',
        }
        code = str(payload.get('code') or '')
        if code in safe_errors:
            return {'type': 'ERROR', 'code': code, 'message': safe_errors[code]}
        return None
    if kind == 'ROBOT_MANUAL_CHANNEL_READY':
        return payload
    # Global heatmaps, layout events, and unknown unscoped payloads could
    # contain fleet information; deny by default for robot-limited operators.
    return None


def websocket_permission(message: dict) -> tuple[tuple[str, ...], str | None]:
    """Return required capabilities for a browser command; unknown writes deny by default."""
    kind = str(message.get('type') or '').upper()
    robot_id = str(message.get('robot_id') or '').strip() or None
    if kind in ('RESYNC', 'SELECT_ROBOT', 'ROBOT_DETAIL_VIEW', 'ROBOT_DETAIL_FRAME_RECEIVED'):
        return ('operator:read',), robot_id
    if kind == 'ROBOT_MODE':
        return ('robot:mode',), robot_id
    if kind == 'MANUAL_ACQUIRE':
        return ('robot:manual',), robot_id
    if kind == 'ROBOT_MANUAL':
        if str(message.get('action') or '').upper() == 'STOP':
            return ('robot:stop', 'robot:manual', 'safety:estop'), robot_id
        return ('robot:manual',), robot_id
    if kind == 'PATH_PREVIEW_REQUEST':
        return ('navigation:preview',), robot_id
    if kind == 'PATH_PREVIEW_INVALIDATE':
        return ('operator:read',), robot_id
    if kind in ('NAV_GOAL',):
        return ('navigation:goal',), robot_id
    if kind in ('NAV_PAUSE', 'NAV_RESUME', 'NAV_CANCEL'):
        return (f'navigation:{kind.split("_", 1)[1].lower()}',), robot_id
    if kind in ('CREATE_TASK', 'ASSIGN_TASK'):
        return ('fleet:dispatch',), robot_id
    if kind == 'ACK_ALERT':
        return ('diagnostics:ack',), None
    if kind in ('SIM_CONTROL', 'INJECT', 'CLEAR_INJECTION'):
        return ('sim:control',), None
    if kind in ('COPILOT_ASK', 'WHATIF_RUN'):
        return ('operator:read',), None
    return ('operator:admin',), robot_id


def authorize_websocket_message(operator: dict | None, message: dict) -> bool:
    permissions, robot_id = websocket_permission(message)
    if str(message.get('type') or '').upper() == 'RESYNC':
        return has_permission(operator, 'operator:read')
    return any(authorized(operator, permission, robot_id) for permission in permissions)


class OperatorSecurityMiddleware:
    """ASGI boundary for both HTTP control APIs and browser WebSockets."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        scope_type = scope.get('type')
        path = str(scope.get('path', ''))
        if scope_type == 'websocket' and path.rstrip('/') == '/ws/ros':
            # ROS bridge has a separate machine token and must never inherit browser identity.
            return await self.app(scope, receive, send)
        if scope_type not in ('http', 'websocket'):
            return await self.app(scope, receive, send)

        protected = mode() == 'PROTECTED_LAN'
        method = str(scope.get('method', 'GET')).upper()
        is_internal_health = (scope_type == 'http' and path.rstrip('/') == '/api/health'
                              and method in SAFE_METHODS and peer_is_loopback(scope))
        source_ok = (peer_is_trusted_proxy(scope) or is_internal_health) if protected else peer_is_loopback(scope)
        if not source_ok:
            return await self._deny(scope_type, send, 403, 'UNTRUSTED_SOURCE', 'request did not arrive from an approved local/proxy address')

        origin = request_origin(scope)
        if origin and not origin_allowed(origin):
            return await self._deny(scope_type, send, 403, 'ORIGIN_REJECTED', 'browser origin is not configured for this HMI')
        if scope_type == 'websocket' and not origin_allowed(origin, required=True):
            return await self._deny(scope_type, send, 4403, 'ORIGIN_REJECTED', 'WebSocket Origin is missing or not configured')

        if protected:
            operator = verify_assertion(scope)
            # Health is a minimal readiness surface for the local runtime supervisor;
            # all other API routes and browser WebSockets require an assertion.
            if operator is None and not is_internal_health:
                return await self._deny(scope_type, send, 401, 'OPERATOR_AUTH_REQUIRED', 'a valid trusted-gateway operator assertion is required')
        else:
            operator = {'identity': 'local-loopback', 'permissions': frozenset({'*'}),
                        'robots': frozenset({'*'}), 'expires_at': float('inf')}

        if scope_type == 'http' and path.startswith('/api/'):
            permission, robot_id = http_permission(path, str(scope.get('method', 'GET')).upper())
            method = str(scope.get('method', 'GET')).upper()
            if protected and method not in SAFE_METHODS and not authorized(operator, permission or 'operator:admin', robot_id):
                return await self._deny(scope_type, send, 403, 'OPERATOR_PERMISSION_REQUIRED', 'operator identity lacks permission or robot scope for this operation')
            if (protected and method in SAFE_METHODS and permission and not is_internal_health
                    and not authorized(operator, permission, robot_id)):
                return await self._deny(scope_type, send, 403, 'OPERATOR_PERMISSION_REQUIRED', 'operator identity lacks read permission or robot scope')
            if protected and method not in SAFE_METHODS:
                log.info('authorized operator HTTP operation identity=%s permission=%s method=%s path=%s',
                         operator['identity'], permission, method, path)

        if scope_type == 'websocket' and path.rstrip('/') == '/ws':
            if protected and not has_permission(operator, 'operator:read'):
                return await self._deny(scope_type, send, 4403, 'OPERATOR_PERMISSION_REQUIRED', 'operator identity lacks WebSocket read permission')
            scope = dict(scope, waretwin_operator=operator)
        return await self.app(scope, receive, send)

    @staticmethod
    async def _deny(scope_type, send, status, code, message):
        if scope_type == 'websocket':
            websocket_code = 4401 if status == 401 else 4403
            await send({'type': 'websocket.close', 'code': websocket_code, 'reason': code})
            return
        import json
        body = json.dumps({'ok': False, 'code': code, 'error': message}).encode('utf-8')
        await send({'type': 'http.response.start', 'status': status,
                    'headers': [(b'content-type', b'application/json'),
                                (b'content-length', str(len(body)).encode('ascii')),
                                (b'cache-control', b'no-store')]})
        await send({'type': 'http.response.body', 'body': body})
