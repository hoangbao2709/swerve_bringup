import asyncio
import ipaddress
import time
from unittest.mock import AsyncMock, patch

from django.test import SimpleTestCase, override_settings

from twin.consumers import TwinConsumer
from twin.operator_security import (
    OperatorSecurityMiddleware,
    http_permission,
    sign_assertion,
    verify_assertion,
    authorize_websocket_message,
)
from twin.runtime import runtime


SECRET = 'test-only-gateway-key-with-32-bytes-minimum'
ORIGIN = 'https://hmi.example.test'
TRUSTED = (ipaddress.ip_network('127.0.0.0/8'),)


class CaptureApp:
    def __init__(self):
        self.scope = None

    async def __call__(self, scope, receive, send):
        self.scope = scope
        if scope['type'] == 'http':
            await send({'type': 'http.response.start', 'status': 204, 'headers': []})
            await send({'type': 'http.response.body', 'body': b''})
        else:
            await send({'type': 'websocket.accept'})


class OperatorSecurityTests(SimpleTestCase):
    def protected_settings(self, **values):
        defaults = {
            'WARETWIN_OPERATOR_AUTH_MODE': 'PROTECTED_LAN',
            'WARETWIN_OPERATOR_GATEWAY_SECRET': SECRET,
            'WARETWIN_OPERATOR_ALLOWED_ORIGINS': (ORIGIN,),
            'WARETWIN_TRUSTED_PROXY_NETWORKS': TRUSTED,
        }
        defaults.update(values)
        return override_settings(**defaults)

    def scope(self, *, method='POST', path='/api/robots/R01/emergency-stop', origin=ORIGIN,
              client=('127.0.0.1', 50000), permissions=None, robots='R01', signed=True,
              expires=None, scope_type='http'):
        expiry = str(expires or int(time.time()) + 30)
        permission_text = ','.join(permissions or ['safety:estop'])
        headers = [(b'host', b'waretwin.internal')]
        if origin:
            headers.append((b'origin', origin.encode('ascii')))
        if signed:
            values = {
                'identity': 'operator-17', 'permissions': permission_text,
                'robots': robots, 'expires': expiry,
            }
            values['signature'] = sign_assertion(method=method, path=path, origin=origin or '',
                identity=values['identity'], permissions=values['permissions'], robots=values['robots'],
                expires=values['expires'], secret=SECRET)
            headers.extend((f'x-waretwin-{key}'.encode('ascii'), value.encode('ascii'))
                           for key, value in values.items())
        result = {'type': scope_type, 'asgi': {'version': '3.0'}, 'path': path,
                  'headers': headers, 'client': client}
        if scope_type == 'http':
            result.update(method=method, scheme='https', http_version='1.1', raw_path=path.encode(),
                          query_string=b'', server=('waretwin.internal', 8000))
        else:
            result.update(subprotocols=[], query_string=b'')
        return result

    def invoke(self, app, scope):
        events = []

        async def receive():
            return {'type': 'http.request', 'body': b'', 'more_body': False}

        async def send(message):
            events.append(message)

        asyncio.run(app(scope, receive, send))
        return events

    def test_unauthenticated_and_forged_rest_control_requests_are_rejected_before_django(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            unsigned = self.invoke(app, self.scope(signed=False))
            forged = self.scope(signed=False)
            forged['headers'].append((b'x-waretwin-identity', b'admin'))
            forged_events = self.invoke(app, forged)
        self.assertEqual(unsigned[0]['status'], 401)
        self.assertEqual(forged_events[0]['status'], 401)
        self.assertIsNone(target.scope)

    def test_permission_and_robot_scope_are_required_for_rest_map_and_safety_actions(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            wrong_permission = self.invoke(app, self.scope(
                path='/api/robots/R01/clear-emergency-stop', permissions=['safety:estop']))
            wrong_robot = self.invoke(app, self.scope(robots='R02'))
            authorized = self.invoke(app, self.scope())
        self.assertEqual(wrong_permission[0]['status'], 403)
        self.assertEqual(wrong_robot[0]['status'], 403)
        self.assertEqual(authorized[0]['status'], 204)
        self.assertEqual(http_permission('/api/robots/R01/local/maps/load', 'POST'), ('map:write', 'R01'))
        self.assertEqual(http_permission('/api/robots/R01/emergency-stop', 'POST'), ('safety:estop', 'R01'))
        self.assertEqual(http_permission('/api/robots/R01/clear-emergency-stop', 'POST'), ('safety:reset', 'R01'))

    def test_robot_affecting_rest_operations_require_their_specific_capability(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        cases = [
            ('/api/robots/R01/local/runtime-mode', 'POST', 'runtime:mode', 'R01'),
            ('/api/robots/R01/local/mapping/start', 'POST', 'map:write', 'R01'),
            ('/api/robots/R01/local/maps/save', 'POST', 'map:write', 'R01'),
            ('/api/robots/R01/local/maps/load', 'POST', 'map:write', 'R01'),
            ('/api/robots/R01/local/maps/resume-session', 'POST', 'map:write', 'R01'),
            ('/api/robots/R01/local/initial-pose', 'POST', 'localization:write', 'R01'),
            ('/api/robots/R01/local/vda5050', 'PUT', 'vda5050:write', 'R01'),
            ('/api/robots/R01/local/vda5050/test', 'POST', 'vda5050:write', 'R01'),
            ('/api/robots/R01/emergency-stop', 'POST', 'safety:estop', 'R01'),
            ('/api/robots/R01/clear-emergency-stop', 'POST', 'safety:reset', 'R01'),
            ('/api/navigation/missions/start', 'POST', 'navigation:goal', None),
            ('/api/navigation/missions/9/pause', 'POST', 'navigation:pause', None),
            ('/api/navigation/missions/9/resume', 'POST', 'navigation:resume', None),
            ('/api/navigation/missions/9/cancel', 'POST', 'navigation:cancel', None),
            ('/api/robots/R01/map-registration', 'POST', 'map:write', 'R01'),
            ('/api/conveyors/C01/command', 'POST', 'robot:control', None),
            ('/api/schedules/create', 'POST', 'fleet:dispatch', None),
            ('/api/admin/users', 'POST', 'operator:admin', None),
        ]
        with self.protected_settings():
            for path, method, permission, robot_id in cases:
                with self.subTest(path=path, method=method):
                    wrong = self.invoke(app, self.scope(path=path, method=method,
                        permissions=['operator:read'], robots='*'))
                    expected_scope = robot_id or '*'
                    valid = self.invoke(app, self.scope(path=path, method=method,
                        permissions=[permission], robots=expected_scope))
                    self.assertEqual(wrong[0]['status'], 403)
                    self.assertEqual(valid[0]['status'], 204)
                    self.assertEqual(http_permission(path, method), (permission, robot_id))

    def test_proxy_source_origin_and_assertion_expiry_are_enforced(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            untrusted_peer = self.invoke(app, self.scope(client=('192.0.2.10', 1234)))
            wrong_origin = self.invoke(app, self.scope(origin='https://attacker.test'))
            expired = self.scope(expires=int(time.time()) - 1)
            expired_events = self.invoke(app, expired)
        self.assertEqual(untrusted_peer[0]['status'], 403)
        self.assertEqual(wrong_origin[0]['status'], 403)
        self.assertEqual(expired_events[0]['status'], 401)

    def test_assertion_is_bound_to_method_and_path_and_duplicate_identity_is_rejected(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            changed_method = self.scope()
            changed_method['method'] = 'GET'
            changed_path = self.scope()
            changed_path['path'] = '/api/robots/R02/emergency-stop'
            changed_path['raw_path'] = b'/api/robots/R02/emergency-stop'
            duplicate = self.scope()
            duplicate['headers'].append((b'x-waretwin-identity', b'forged'))
            events = [self.invoke(app, item) for item in (changed_method, changed_path, duplicate)]
        self.assertEqual([item[0]['status'] for item in events], [401, 401, 401])
        self.assertIsNone(target.scope)

    def test_only_loopback_health_probe_is_available_without_a_gateway_assertion(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            internal = self.invoke(app, self.scope(method='GET', path='/api/health/', signed=False))
            remote = self.invoke(app, self.scope(method='GET', path='/api/health/', signed=False,
                                                 client=('192.0.2.10', 1234)))
        self.assertEqual(internal[0]['status'], 204)
        self.assertEqual(remote[0]['status'], 403)

    def test_browser_websocket_requires_signed_origin_and_ros_bridge_stays_separate(self):
        target = CaptureApp()
        app = OperatorSecurityMiddleware(target)
        with self.protected_settings():
            unsigned = self.invoke(app, self.scope(scope_type='websocket', path='/ws', method='GET', signed=False))
            accepted = self.invoke(app, self.scope(scope_type='websocket', path='/ws', method='GET',
                                                   permissions=['operator:read'], robots='*'))
            bridge = self.invoke(app, self.scope(scope_type='websocket', path='/ws/ros', method='GET',
                                                 origin='', signed=False, client=('192.0.2.10', 4000)))
        self.assertEqual(unsigned[0]['code'], 4401)
        self.assertEqual(accepted[0]['type'], 'websocket.accept')
        self.assertNotIn('waretwin_operator', target.scope)
        self.assertEqual(bridge[0]['type'], 'websocket.accept')
        self.assertNotIn('waretwin_operator', target.scope)

    def test_websocket_command_permissions_fail_before_runtime_dispatch(self):
        operator = {'identity': 'operator-17', 'permissions': frozenset({'operator:read'}),
                    'robots': frozenset({'R01'}), 'expires_at': time.time() + 30}
        messages = [
            {'type': 'MANUAL_ACQUIRE', 'robot_id': 'R01', 'lease_id': 'a' * 32},
            {'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'FORWARD'},
            {'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'MANUAL'},
            {'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 1, 'y': 2, 'yaw': 0},
        ]
        with self.protected_settings():
            self.assertTrue(all(not authorize_websocket_message(operator, message) for message in messages))
            self.assertFalse(authorize_websocket_message({**operator, 'robots': frozenset({'R02'})}, messages[0]))
            authorized = {**operator, 'permissions': frozenset({'robot:manual', 'robot:mode',
                'navigation:goal'}), 'expires_at': time.time() + 30}
            self.assertTrue(authorize_websocket_message(authorized, messages[0]))
            self.assertTrue(authorize_websocket_message(authorized, messages[3]))
            self.assertFalse(authorize_websocket_message({**authorized, 'expires_at': time.time() - 1}, messages[0]))
            self.assertTrue(authorize_websocket_message({**operator, 'permissions': frozenset({'safety:estop'})},
                                                        {'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP'}))

        consumer = TwinConsumer()
        consumer.operator = operator
        consumer.scope = {'waretwin_operator': operator}
        consumer.control_only = False
        consumer._manual_window_started = time.monotonic()
        consumer._manual_window_count = 0
        consumer._message_window_started = time.monotonic()
        consumer._message_window_count = 0
        consumer.send_json = AsyncMock()
        with self.protected_settings(), patch.object(runtime, 'handle_message', new=AsyncMock()) as dispatch:
            asyncio.run(consumer.receive_json(messages[1]))
        dispatch.assert_not_awaited()
        consumer.send_json.assert_awaited_once()
        self.assertEqual(consumer.send_json.await_args.args[0]['code'], 'OPERATOR_PERMISSION_REQUIRED')
