import json
from copy import deepcopy
import tempfile
from pathlib import Path
from unittest.mock import patch

from django.contrib.auth.models import User
from django.test import Client, TestCase, override_settings

from accounts.models import ApiToken
from twin.local_control import (
    get_robot_map,
    list_robot_maps,
    map_output_prefix,
    register_saved_map,
    validate_map_name,
)
from twin.models import RobotVda5050Configuration
from twin.runtime import runtime
from twin.vda5050 import (
    _on_order,
    _mqtt_client,
    _subscribe,
    decrypt_password,
    encrypt_password,
    public_configuration,
    validate_configuration,
)


class LocalMapRegistryTests(TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.settings = override_settings(WARETWIN_ARTIFACT_ROOT=Path(self.tempdir.name))
        self.settings.enable()

    def tearDown(self):
        self.settings.disable()
        self.tempdir.cleanup()

    def test_names_are_safe_and_robot_scoped_maps_never_expose_host_paths(self):
        with self.assertRaises(ValueError):
            validate_map_name('../escape')
        with self.assertRaises(ValueError):
            map_output_prefix('../R02', 'floor')
        prefix = map_output_prefix('R01', 'floor_1')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        prefix.with_suffix('.yaml').write_text(
            'image: floor_1.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0.0]\n',
            encoding='utf-8',
        )

        record = register_saved_map('R01', 'floor_1', prefix.with_suffix('.yaml'))
        self.assertEqual(record['robot_id'], 'R01')
        self.assertEqual(record['origin'], [1.0, 2.0, 0.0])
        self.assertNotIn('yaml_path', record)
        self.assertEqual(list_robot_maps('R01'), [record])
        _stored, yaml_path, image_path = get_robot_map('R01', record['id'])
        self.assertTrue(yaml_path.is_file())
        self.assertTrue(image_path.is_file())
        with self.assertRaises(FileNotFoundError):
            get_robot_map('R02', record['id'])

    def test_duplicate_map_names_are_rejected(self):
        prefix = map_output_prefix('R01', 'same')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        prefix.with_suffix('.yaml').write_text(
            'image: same.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n', encoding='utf-8')
        register_saved_map('R01', 'same', prefix.with_suffix('.yaml'))
        with self.assertRaises(FileExistsError):
            register_saved_map('R01', 'same', prefix.with_suffix('.yaml'))


class Vda5050ConfigurationTests(TestCase):
    def test_secret_is_encrypted_and_never_returned(self):
        secret = 'warehouse-secret-42'
        encrypted = encrypt_password(secret)
        self.assertNotEqual(encrypted, secret)
        self.assertEqual(decrypt_password(encrypted), secret)
        config = RobotVda5050Configuration(
            robot_id='R01', mqtt_password_ciphertext=encrypted, mqtt_host='broker.local')
        public = public_configuration(config)
        self.assertTrue(public['password_configured'])
        self.assertNotIn('mqtt_password', public)
        self.assertNotIn('mqtt_password_ciphertext', public)
        self.assertFalse(public['allow_instant_actions'])
        self.assertFalse(public['instant_actions_supported'])
        self.assertEqual(public['instant_actions_status'], 'NOT_IMPLEMENTED')
        self.assertNotIn(secret, json.dumps(public))

    def test_configuration_validation_rejects_bad_ports_host_and_boolean_coercion(self):
        with self.assertRaises(ValueError):
            validate_configuration({'enabled': True, 'mqtt_host': 'mqtt://broker', 'mqtt_port': 1883})
        with self.assertRaises(ValueError):
            validate_configuration({'enabled': True, 'mqtt_host': 'broker.local', 'mqtt_port': 65536})
        with self.assertRaises(ValueError):
            validate_configuration({'enabled': True, 'mqtt_host': 'broker.local', 'allow_task': 'false'})
        with self.assertRaises(ValueError):
            validate_configuration({'enabled': True, 'mqtt_host': 'broker.local', 'topic_prefix': 'vda5050/unsafe topic'})
        result = validate_configuration({
            'enabled': True, 'mqtt_host': 'broker.local', 'mqtt_port': 1883,
            'allow_task': False,
        })
        self.assertFalse(result['allow_task'])

    def test_auto_reconnect_setting_controls_the_paho_reconnect_policy(self):
        config = type('Config', (), {
            'robot_id': 'R01', 'client_id': '', 'mqtt_protocol_version': '3.1.1',
            'mqtt_username': '', 'tls_enabled': False, 'reconnect_interval': 5,
            'auto_reconnect': False,
        })()
        client = _mqtt_client(config, '')
        self.assertFalse(client._reconnect_on_failure)
        config.auto_reconnect = True
        client = _mqtt_client(config, '')
        self.assertTrue(client._reconnect_on_failure)

    def test_allow_task_controls_order_subscription_and_drops_queued_orders(self):
        class CaptureClient:
            subscriptions = None
            def subscribe(self, topics):
                self.subscriptions = topics

        config = type('Config', (), {
            'robot_id': 'R01', 'protocol_version': '2.0.0', 'topic_prefix': 'vda5050',
            'interface_name': 'uagv', 'manufacturer': 'PTAGV', 'serial_number': 'R01',
            'allow_task': False, 'allow_instant_actions': True,
        })()
        client = CaptureClient()
        _subscribe(client, config)
        self.assertIsNone(client.subscriptions)
        config.allow_task = True
        _subscribe(client, config)
        self.assertEqual(len(client.subscriptions), 1)
        self.assertTrue(client.subscriptions[0][0].endswith('/order'))
        self.assertFalse(any(topic.endswith('/instantActions') for topic, _qos in client.subscriptions))
        config.allow_task = False
        message = type('Message', (), {'topic': 'vda5050/uagv/v2/PTAGV/R01/order', 'payload': b'{"orderId":"blocked"}'})()
        with patch('twin.ros_bridge_consumer.registry.send') as send:
            _on_order(config, client, message)
        send.assert_not_called()

        # A callback from the old client can race with an Apply operation.
        # It must use the newly applied robot policy, not the stale model
        # object captured when that MQTT client subscribed.
        config.allow_task = True
        with patch.dict('twin.vda5050._task_policy', {'R01': (False, True)}):
            with patch('twin.ros_bridge_consumer.registry.send') as send:
                _on_order(config, client, message)
            send.assert_not_called()


class LocalControlApiTests(TestCase):
    def setUp(self):
        self.client = Client()
        user = User.objects.create_user(username='local-control-test', password='test-only-password')
        token = ApiToken.issue(user)
        self.client.defaults['HTTP_AUTHORIZATION'] = f'Bearer {token.key}'
        robots = runtime.engine.state.setdefault('robots', {})
        self.previous_robot = deepcopy(robots.get('R01'))
        robots['R01'] = {
            **(self.previous_robot or {}), 'control_mode': 'MANUAL',
            'navigation_state': 'MANUAL', 'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
        }
        self.old_runtime_mode = runtime.runtime_mode
        self.old_operation_mode = runtime.operation_mode
        runtime.runtime_mode = 'GAZEBO_ROS'
        runtime.operation_mode = 'MAPPING'
        self.bridge_patch = patch.object(runtime, 'robot_bridge_online', return_value=True)
        self.bridge_patch.start()

    def tearDown(self):
        self.bridge_patch.stop()
        runtime.runtime_mode = self.old_runtime_mode
        runtime.operation_mode = self.old_operation_mode
        runtime.local_map_overrides.clear()
        runtime.local_map_transitions.clear()
        runtime.robot_mapping_state.clear()
        robots = runtime.engine.state.setdefault('robots', {})
        if self.previous_robot is None:
            robots.pop('R01', None)
        else:
            robots['R01'] = self.previous_robot

    def test_mapping_start_stop_and_map_save_wait_for_bridge_results_and_files(self):
        with patch('twin.local_control_views._bridge_request', return_value={'ok': True, 'result': {'mapping_state': 'MAPPING'}}) as request:
            started = self.client.post('/api/robots/R01/local/mapping/start', data='{}', content_type='application/json')
            self.assertEqual(started.status_code, 200)
            self.assertEqual(started.json()['mapping_state'], 'MAPPING')
            self.assertEqual(request.call_args.args[1], 'MAPPING_START')

        with patch('twin.local_control_views._bridge_request', return_value={'ok': True, 'result': {'mapping_state': 'PAUSED'}}) as request:
            stopped = self.client.post('/api/robots/R01/local/mapping/stop', data='{}', content_type='application/json')
            self.assertEqual(stopped.status_code, 200)
            self.assertEqual(request.call_args.args[1], 'MAPPING_STOP')

        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            def save_result(robot_id, operation, payload, timeout):
                prefix = Path(payload['output_prefix'])
                prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
                prefix.with_suffix('.yaml').write_text(
                    f'image: {prefix.name}.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n', encoding='utf-8')
                return {'ok': True, 'result': {'yaml_path': str(prefix.with_suffix('.yaml'))}}
            with patch('twin.local_control_views._bridge_request', side_effect=save_result) as request:
                response = self.client.post('/api/robots/R01/local/maps/save',
                                            data=json.dumps({'name': 'floor_1'}), content_type='application/json')
                self.assertEqual(response.status_code, 200, response.content)
                self.assertTrue(Path(request.call_args.args[2]['output_prefix'] + '.yaml').is_file())
                self.assertEqual(response.json()['map']['name'], 'floor_1')

    def test_load_map_and_initial_pose_route_to_the_robot_bridge(self):
        self.old_operation_mode = runtime.operation_mode
        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            prefix = map_output_prefix('R01', 'floor')
            prefix.with_suffix('.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes(10_000))
            prefix.with_suffix('.yaml').write_text('image: floor.pgm\nresolution: 0.1\norigin: [-5, -5, 0]\n', encoding='utf-8')
            map_record = register_saved_map('R01', 'floor', prefix.with_suffix('.yaml'))
            runtime.operation_mode = 'NAVIGATION'
            with patch('twin.local_control_views._bridge_request', side_effect=lambda _robot, _operation, payload, **_kwargs: {
                'ok': True, 'result': {'map_id': payload['map_id'], 'active_map_revision': payload['map_revision']},
            }) as request:
                loaded = self.client.post('/api/robots/R01/local/maps/load',
                                          data=json.dumps({'map_id': map_record['id']}), content_type='application/json')
                self.assertEqual(loaded.status_code, 200, loaded.content)
                self.assertEqual(request.call_args.args[1], 'MAP_LOAD')
                self.assertEqual(runtime.local_map_overrides['R01'], map_record['id'])

            runtime.engine.state['robots']['R01'].update({'control_mode': 'MANUAL', 'navigation_state': 'IDLE',
                                                          'vx': 0.0, 'vy': 0.0, 'wz': 0.0})
            with patch('twin.local_control_views._bridge_request', side_effect=lambda _robot, _operation, payload, **_kwargs: {
                'ok': True, 'result': {'runtime_pose_confirmed': True,
                                       'active_map_id': payload['active_map_id'],
                                       'active_map_revision': payload['active_map_revision']},
            }) as request:
                applied = self.client.post('/api/robots/R01/local/initial-pose',
                                           data=json.dumps({'x': 2.0, 'y': 3.0, 'yaw': 1.2, 'frame_id': 'map'}),
                                           content_type='application/json')
                self.assertEqual(applied.status_code, 200, applied.content)
                self.assertEqual(request.call_args.args[1], 'INITIAL_POSE')
                self.assertEqual(request.call_args.args[2]['x'], 2.0)

    def test_map_load_and_initial_pose_require_manual_stopped_robot(self):
        robot = runtime.engine.state['robots']['R01']
        robot['control_mode'] = 'AUTONOMOUS'
        with patch('twin.local_control_views._bridge_request') as request:
            response = self.client.post(
                '/api/robots/R01/local/initial-pose',
                data=json.dumps({'x': 2.0, 'y': 3.0, 'yaw': 1.2, 'frame_id': 'map'}),
                content_type='application/json')
        self.assertEqual(response.status_code, 409)
        request.assert_not_called()
        robot['control_mode'] = 'MANUAL'
        robot['vx'] = 0.2
        with patch('twin.local_control_views._bridge_request') as request:
            response = self.client.post(
                '/api/robots/R01/local/initial-pose',
                data=json.dumps({'x': 2.0, 'y': 3.0, 'yaw': 1.2, 'frame_id': 'map'}),
                content_type='application/json')
        self.assertEqual(response.status_code, 409)
        request.assert_not_called()

    def test_runtime_mode_switch_requires_manual_stop_and_queues_supervised_restart(self):
        robots = runtime.engine.state.setdefault('robots', {})
        previous_robot = deepcopy(robots.get('R01'))
        robots['R01'] = {**(previous_robot or {}), 'control_mode': 'MANUAL', 'navigation_state': 'MANUAL'}
        runtime.operation_mode = 'NAVIGATION'
        try:
            with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_STACK_RUNTIME_DIR=Path(tempdir)):
                status_path = Path(tempdir) / 'mode-switch-status.json'
                status_path.write_text(json.dumps({'robot_id': 'R01', 'mode': 'navigation', 'status': 'READY'}))
                robots['R01']['control_mode'] = 'AUTONOMOUS'
                with patch.object(runtime, 'robot_bridge_online', return_value=True):
                    blocked = self.client.post(
                        '/api/robots/R01/local/runtime-mode',
                        data=json.dumps({'mode': 'MAPPING'}), content_type='application/json')
                self.assertEqual(blocked.status_code, 409)
                self.assertFalse((Path(tempdir) / 'mode-switch-request.json').exists())

                robots['R01']['control_mode'] = 'MANUAL'
                with patch.object(runtime, 'robot_bridge_online', return_value=True):
                    response = self.client.post(
                        '/api/robots/R01/local/runtime-mode',
                        data=json.dumps({'mode': 'MAPPING'}), content_type='application/json')
                self.assertEqual(response.status_code, 202, response.content)
                self.assertEqual(response.json()['requested_mode'], 'MAPPING')
                queued = json.loads((Path(tempdir) / 'mode-switch-request.json').read_text())
                self.assertEqual(queued['robot_id'], 'R01')
                self.assertEqual(queued['mode'], 'mapping')
                self.assertEqual(json.loads(status_path.read_text())['status'], 'REQUESTED')
        finally:
            if previous_robot is None:
                robots.pop('R01', None)
            else:
                robots['R01'] = previous_robot

    def test_real_robot_mode_change_requires_a_configured_physical_runtime_adapter(self):
        old_mode, old_operation_mode = runtime.runtime_mode, runtime.operation_mode
        runtime.runtime_mode = 'REAL_ROBOT'
        runtime.operation_mode = 'NAVIGATION'
        runtime.engine.state['robots']['R01'].update({
            'control_mode': 'MANUAL', 'navigation_state': 'MANUAL',
            'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
        })
        try:
            with tempfile.TemporaryDirectory() as tempdir, \
                    override_settings(WARETWIN_REAL_RUNTIME_ADAPTER='',
                                     WARETWIN_STACK_RUNTIME_DIR=Path(tempdir)), \
                    patch.object(runtime, 'robot_bridge_online', return_value=True):
                response = self.client.post(
                    '/api/robots/R01/local/runtime-mode',
                    data=json.dumps({'mode': 'MAPPING'}), content_type='application/json')
            self.assertEqual(response.status_code, 501, response.content)
            self.assertEqual(response.json()['error']['code'], 'HTTP_501')
            self.assertEqual(response.json()['error']['details']['code'], 'REAL_RUNTIME_ADAPTER_UNAVAILABLE')
            self.assertIn('no Gazebo', response.json()['error']['message'])
        finally:
            runtime.runtime_mode, runtime.operation_mode = old_mode, old_operation_mode

    def test_vda_api_persists_secret_and_test_result_without_returning_secret(self):
        body = {
            'enabled': False, 'mqtt_host': 'broker.local', 'mqtt_port': 1883,
            'mqtt_username': 'robot01', 'mqtt_password': 'warehouse-secret-42',
            'allow_task': False, 'allow_instant_actions': True,
        }
        with patch('twin.local_control_views.apply_configuration', return_value={'status': 'DISABLED'}):
            saved = self.client.put('/api/robots/R01/local/vda5050', data=json.dumps(body), content_type='application/json')
        self.assertEqual(saved.status_code, 200, saved.content)
        config = RobotVda5050Configuration.objects.get(robot_id='R01')
        self.assertNotEqual(config.mqtt_password_ciphertext, body['mqtt_password'])
        self.assertFalse(config.allow_task)
        self.assertFalse(config.allow_instant_actions)
        readback = self.client.get('/api/robots/R01/local/vda5050').json()
        self.assertTrue(readback['password_configured'])
        self.assertFalse(readback['allow_instant_actions'])
        self.assertNotIn(body['mqtt_password'], json.dumps(readback))

        with patch('twin.local_control_views.test_connection', return_value={
            'ok': True, 'latency_ms': 12.4, 'broker': 'broker.local:1883', 'error_code': None, 'message': None,
        }) as test_connection:
            tested = self.client.post('/api/robots/R01/local/vda5050/test', data='{}', content_type='application/json')
        self.assertEqual(tested.status_code, 200, tested.content)
        self.assertTrue(tested.json()['ok'])
        self.assertNotIn(body['mqtt_password'], json.dumps(tested.json()))
        test_connection.assert_called_once()

    def test_vda_save_apply_failure_rolls_persisted_configuration_back(self):
        body = {
            'enabled': True, 'mqtt_host': 'broker.local', 'mqtt_port': 1883,
            'mqtt_username': '', 'mqtt_password': '', 'allow_task': False,
        }
        with patch('twin.local_control_views.apply_configuration', side_effect=[
            {'status': 'DISCONNECTED', 'last_error': 'broker refused connection'},
            {'status': 'DISABLED'},
        ]) as apply:
            response = self.client.put('/api/robots/R01/local/vda5050', data=json.dumps(body),
                                       content_type='application/json')
        self.assertEqual(response.status_code, 502)
        self.assertEqual(response.json()['error']['code'], 'VDA5050_APPLY_FAILED')
        config = RobotVda5050Configuration.objects.get(robot_id='R01')
        self.assertFalse(config.enabled)
        self.assertEqual(config.mqtt_host, '')
        self.assertTrue(config.allow_task)
        self.assertEqual(apply.call_count, 2)

    def test_vda_read_restores_a_persisted_enabled_robot_configuration(self):
        config = RobotVda5050Configuration.objects.create(
            robot_id='R01', enabled=True, mqtt_host='broker.local', mqtt_port=1883)
        state = {'status': 'CONNECTED', 'last_error': None, 'ignored_orders': 0}
        with patch('twin.local_control_views.ensure_configuration_active', return_value=state) as ensure:
            response = self.client.get('/api/robots/R01/local/vda5050')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()['connection_status'], 'CONNECTED')
        ensure.assert_called_once_with(config)
