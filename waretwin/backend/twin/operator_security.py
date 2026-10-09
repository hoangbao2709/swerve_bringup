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
    if not operator or float(operator.get('expires_at', 0)) <= time.time():
        return False
    permissions = operator.get('permissions', frozenset())
    if 'operator:admin' not in permissions and permission not in permissions:
        return False
    robots = operator.get('robots', frozenset())
    if robot_id:
        return '*' in robots or robot_id in robots
    return '*' in robots


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
            if protected and not authorized(operator, 'operator:read'):
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
