import json
import base64
import math
from copy import deepcopy
import tempfile
import zlib
from pathlib import Path
from unittest.mock import patch

from django.contrib.auth.models import User
from django.http import JsonResponse
from django.test import Client, TestCase, override_settings

from accounts.models import ApiToken
from twin.local_control import (
    get_robot_map,
    get_robot_slam_session,
    list_robot_maps,
    map_output_prefix,
    register_saved_map,
    slam_map_restoration_evidence,
    validate_map_name,
)
from twin.models import RobotVda5050Configuration
from twin.runtime import runtime
from twin.local_control_views import SimulationRuntimeAdapter
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

    def _restoration_fixture(self, name, width, height, *, saved_occupied=(), saved_free=(),
                             live_width=None, live_height=None, live_origin=(0, 0, 0),
                             live_resolution=None, live_occupied=(), live_free=(),
                             saved_resolution=0.05):
        prefix = map_output_prefix('R01', name)
        pixels = bytearray([205] * (width * height))
        for row, col in saved_free:
            pixels[(height - row - 1) * width + col] = 254
        for row, col in saved_occupied:
            pixels[(height - row - 1) * width + col] = 0
        prefix.with_suffix('.pgm').write_bytes(
            f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels))
        prefix.with_suffix('.yaml').write_text(
            f'image: {name}.pgm\nresolution: {saved_resolution}\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        record = register_saved_map('R01', name, prefix.with_suffix('.yaml'))
        live_width = width if live_width is None else live_width
        live_height = height if live_height is None else live_height
        live_resolution = saved_resolution if live_resolution is None else live_resolution
        encoded = bytearray([0] * (live_width * live_height))
        for row, col in live_free:
            encoded[row * live_width + col] = 1
        for row, col in live_occupied:
            encoded[row * live_width + col] = 101
        known_cells = sum(value != 0 for value in encoded)
        map_data = {
            'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'resume-test-session',
            'width': live_width, 'height': live_height, 'resolution': live_resolution,
            'known_cells': known_cells,
            'origin': {'x': live_origin[0], 'y': live_origin[1], 'yaw': live_origin[2]},
            'data_encoding': 'zlib-base64-offset1',
            'data_zlib_base64': base64.b64encode(zlib.compress(encoded)).decode('ascii'),
        }
        evidence = slam_map_restoration_evidence(
            record, prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'), map_data)
        return evidence

    def test_names_are_safe_and_robot_scoped_maps_never_expose_host_paths(self):
        with self.assertRaises(ValueError):
            validate_map_name('../escape')
        with self.assertRaises(ValueError):
            map_output_prefix('../R02', 'floor')
        prefix = map_output_prefix('R01', 'floor_1')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        prefix.with_suffix('.yaml').write_text(
            'image: floor_1.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
            encoding='utf-8',
        )

        record = register_saved_map('R01', 'floor_1', prefix.with_suffix('.yaml'))
        self.assertEqual(record['robot_id'], 'R01')
        self.assertEqual(record['origin'], [1.0, 2.0, 0.0])
        self.assertEqual(record['map_id'], record['id'])
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
            'image: same.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        register_saved_map('R01', 'same', prefix.with_suffix('.yaml'))
        with self.assertRaises(FileExistsError):
            register_saved_map('R01', 'same', prefix.with_suffix('.yaml'))

    def test_registry_requires_valid_nav2_occupancy_metadata(self):
        prefix = map_output_prefix('R01', 'bad_thresholds')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        prefix.with_suffix('.yaml').write_text(
            'image: bad_thresholds.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.2\nfree_thresh: 0.3\n', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'occupancy thresholds'):
            register_saved_map('R01', 'bad_thresholds', prefix.with_suffix('.yaml'))

    def test_registry_keeps_navigation_map_and_resumable_slam_session_as_separate_local_products(self):
        prefix = map_output_prefix('R01', 'warehouse')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        prefix.with_suffix('.yaml').write_text(
            'image: warehouse.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        posegraph = prefix.parent / 'warehouse_slam_session.posegraph'
        session_data = prefix.parent / 'warehouse_slam_session.data'
        posegraph.write_bytes(b'pose graph')
        session_data.write_bytes(b'sensor data')
        record = register_saved_map(
            'R01', 'warehouse', prefix.with_suffix('.yaml'),
            mapping_metadata={
                'width': 1, 'height': 1, 'resolution': 0.05,
                'known_cells': 1, 'unknown_cells': 0, 'free_cells': 1,
                'occupied_cells': 0, 'explored_area_m2': 0.0025,
                'mapping_session_id': 'session-1', 'map_version': 3,
            },
            slam_session={
                'engine': 'SLAM_TOOLBOX', 'status': 'AVAILABLE',
                'posegraph_path': str(posegraph), 'data_path': str(session_data),
            },
        )
        self.assertEqual(record['map_kind'], 'SAVED_LOCAL_MAP')
        self.assertFalse(record['canonical_map_promoted'])
        self.assertEqual(record['navigation_artifacts'], {'yaml': True, 'image': True, 'image_format': '.pgm'})
        self.assertEqual(record['slam_session_state']['status'], 'AVAILABLE')
        self.assertEqual(record['known_cells'], 1)
        self.assertEqual(record['explored_area_m2'], 0.0025)
        self.assertNotIn('yaml_path', record)
        self.assertNotIn('posegraph_path', record)
        self.assertEqual(list_robot_maps('R01'), [record])
        session_data.unlink()
        rows = list_robot_maps('R01')
        self.assertEqual(rows[0]['slam_session_state']['status'], 'MISSING')
        self.assertTrue(get_robot_map('R01', record['id'])[1].is_file())

    def test_saved_slam_session_resolves_only_complete_robot_scoped_files(self):
        prefix = map_output_prefix('R01', 'resume')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
        prefix.with_suffix('.yaml').write_text(
            'image: resume.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        session_prefix = prefix.parent / 'resume_slam_session'
        session_prefix.with_suffix('.posegraph').write_bytes(b'pose graph')
        session_prefix.with_suffix('.data').write_bytes(b'sensor data')
        record = register_saved_map('R01', 'resume', prefix.with_suffix('.yaml'), slam_session={
            'engine': 'SLAM_TOOLBOX', 'status': 'AVAILABLE',
            'posegraph_path': str(session_prefix.with_suffix('.posegraph')),
            'data_path': str(session_prefix.with_suffix('.data')),
        })
        loaded, session, posegraph, session_data = get_robot_slam_session('R01', record['id'])
        self.assertEqual(loaded['id'], record['id'])
        self.assertEqual(session, session_prefix)
        self.assertEqual(posegraph, session_prefix.with_suffix('.posegraph'))
        self.assertEqual(session_data, session_prefix.with_suffix('.data'))

    def test_restoration_gate_compares_saved_pgm_cells_against_live_slam_grid(self):
        prefix = map_output_prefix('R01', 'restore')
        prefix.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 254, 205, 254]))
        prefix.with_suffix('.yaml').write_text(
            'image: restore.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        record = register_saved_map('R01', 'restore', prefix.with_suffix('.yaml'))
        # OccupancyGrid row zero is the bottom: unknown/free, then occupied/free.
        encoded = base64.b64encode(zlib.compress(bytes([0, 1, 101, 1]))).decode('ascii')
        evidence = slam_map_restoration_evidence(record, prefix.with_suffix('.yaml'),
            prefix.with_suffix('.pgm'), {
                'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'resumed-1',
                'width': 2, 'height': 2, 'resolution': 0.1, 'known_cells': 3,
                'origin': {'x': 0, 'y': 0, 'yaw': 0},
                'data_encoding': 'zlib-base64-offset1', 'data_zlib_base64': encoded,
            })
        self.assertTrue(evidence['passed'], evidence)
        self.assertEqual(evidence['saved_known_cells'], 3)
        self.assertEqual(evidence['matched_saved_cells'], 3)
        self.assertEqual(evidence['cell_class_agreement_ratio'], 1.0)

    def test_restoration_preserves_identical_occupancy_grid(self):
        evidence = self._restoration_fixture(
            'identical_grid', 2, 2, saved_occupied={(0, 0)}, saved_free={(0, 1)},
            live_occupied={(0, 0)}, live_free={(0, 1)})
        self.assertTrue(evidence['passed'], evidence)
        self.assertEqual(evidence['saved_occupied_cells'], 1)
        self.assertEqual(evidence['matched_occupied_cells'], 1)
        self.assertEqual(evidence['missing_occupied_cells'], 0)

    def test_restoration_matches_same_geometry_after_subcell_origin_shift(self):
        evidence = self._restoration_fixture(
            'subcell_origin', 2, 2, saved_occupied={(0, 0)}, saved_free={(0, 1)},
            live_origin=(0.02, 0.02, 0), live_occupied={(0, 0)}, live_free={(0, 1)})
        self.assertTrue(evidence['passed'], evidence)
        self.assertEqual(evidence['matched_occupied_cells'], 1)
        self.assertLessEqual(evidence['max_occupied_match_distance_m'], math.sqrt(2) * 0.05 / 2)

    def test_restoration_matches_boundary_occupied_cell_in_neighboring_live_cell(self):
        evidence = self._restoration_fixture(
            'boundary_neighbor', 1, 1, saved_occupied={(0, 0)},
            live_origin=(0.015, -0.03, 0), live_occupied={(0, 0)})
        self.assertTrue(evidence['passed'], evidence)
        self.assertEqual(evidence['covered_saved_cells'], 1)
        self.assertEqual(evidence['matched_occupied_cells'], 1)
        self.assertLessEqual(evidence['max_occupied_match_distance_m'], math.sqrt(2) * 0.05 / 2)

    def test_restoration_fails_when_occupied_wall_is_deleted(self):
        evidence = self._restoration_fixture(
            'deleted_wall', 1, 1, saved_occupied={(0, 0)}, live_width=1, live_height=1)
        self.assertFalse(evidence['passed'])
        self.assertEqual(evidence['missing_occupied_cells'], 1)

    def test_restoration_fails_when_occupied_becomes_free(self):
        evidence = self._restoration_fixture(
            'occupied_to_free', 2, 1, saved_occupied={(0, 0)}, saved_free={(0, 1)},
            live_free={(0, 0), (0, 1)})
        self.assertFalse(evidence['passed'])
        self.assertEqual(evidence['matched_occupied_cells'], 0)
        self.assertEqual(evidence['missing_occupied_cells'], 1)

    def test_restoration_fails_when_occupied_becomes_unknown_without_neighboring_occupied(self):
        evidence = self._restoration_fixture(
            'occupied_to_unknown', 1, 1, saved_occupied={(0, 0)})
        self.assertFalse(evidence['passed'])
        self.assertEqual(evidence['matched_occupied_cells'], 0)
        self.assertEqual(evidence['missing_occupied_cells'], 1)

    def test_restoration_rejects_incompatible_map_resolution(self):
        evidence = self._restoration_fixture(
            'incompatible_resolution', 1, 1, saved_occupied={(0, 0)},
            live_resolution=0.1, live_occupied={(0, 0)})
        self.assertFalse(evidence['passed'])
        self.assertIn('not compatible', evidence['reason'])

    def test_restoration_rejects_large_unregistered_origin_shift(self):
        evidence = self._restoration_fixture(
            'large_origin_shift', 1, 1, saved_occupied={(0, 0)},
            live_origin=(0.2, 0, 0), live_occupied={(0, 0)})
        self.assertFalse(evidence['passed'])
        self.assertEqual(evidence['matched_occupied_cells'], 0)
        self.assertEqual(evidence['missing_occupied_cells'], 1)


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
        self.old_map_snapshots = runtime.robot_map_snapshots.copy()
        self.old_mapping_sessions = runtime.robot_mapping_sessions.copy()
        self.old_local_map_overrides = runtime.local_map_overrides.copy()
        self.old_local_map_revisions = runtime.local_map_revisions.copy()
        self.old_local_map_status = runtime.local_map_status.copy()
        self.old_pending_local_map_loads = runtime.pending_local_map_loads.copy()
        self.old_pending_slam_session_resumes = runtime.pending_slam_session_resumes.copy()
        runtime.runtime_mode = 'GAZEBO_ROS'
        runtime.operation_mode = 'MAPPING'
        runtime.robot_mapping_sessions['R01'] = 'unit-session'
        runtime.robot_map_snapshots['R01'] = {'map': {
            'robot_id': 'R01', 'map_source': 'SLAM_TOOLBOX',
            'mapping_session_id': 'unit-session', 'map_version': 3,
            'width': 1, 'height': 1, 'resolution': 0.05,
            'known_cells': 1, 'unknown_cells': 0, 'free_cells': 1,
            'occupied_cells': 0, 'explored_area_m2': 0.0025,
        }}
        self.bridge_patch = patch.object(runtime, 'robot_bridge_online', return_value=True)
        self.bridge_patch.start()

    def tearDown(self):
        self.bridge_patch.stop()
        runtime.runtime_mode = self.old_runtime_mode
        runtime.operation_mode = self.old_operation_mode
        runtime.robot_map_snapshots.clear(); runtime.robot_map_snapshots.update(self.old_map_snapshots)
        runtime.robot_mapping_sessions.clear(); runtime.robot_mapping_sessions.update(self.old_mapping_sessions)
        runtime.local_map_overrides.clear(); runtime.local_map_overrides.update(self.old_local_map_overrides)
        runtime.local_map_revisions.clear(); runtime.local_map_revisions.update(self.old_local_map_revisions)
        runtime.local_map_status.clear(); runtime.local_map_status.update(self.old_local_map_status)
        runtime.local_map_transitions.clear()
        runtime.pending_local_map_loads.clear(); runtime.pending_local_map_loads.update(self.old_pending_local_map_loads)
        runtime.pending_slam_session_resumes.clear(); runtime.pending_slam_session_resumes.update(self.old_pending_slam_session_resumes)
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
                session_prefix = Path(payload['session_output_prefix'])
                prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
                prefix.with_suffix('.yaml').write_text(
                    f'image: {prefix.name}.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
                session_prefix.with_suffix('.posegraph').write_bytes(b'pose graph')
                session_prefix.with_suffix('.data').write_bytes(b'sensor data')
                return {'ok': True, 'result': {
                    'yaml_path': str(prefix.with_suffix('.yaml')),
                    'slam_session': {
                        'engine': 'SLAM_TOOLBOX', 'status': 'AVAILABLE',
                        'posegraph_path': str(session_prefix.with_suffix('.posegraph')),
                        'data_path': str(session_prefix.with_suffix('.data')),
                    },
                }}
            runtime.robot_mapping_state['R01'] = 'PAUSED'
            with patch('twin.local_control_views._bridge_request', side_effect=save_result) as request:
                response = self.client.post('/api/robots/R01/local/maps/save',
                                            data=json.dumps({'name': 'floor_1'}), content_type='application/json')
                self.assertEqual(response.status_code, 200, response.content)
                self.assertTrue(Path(request.call_args.args[2]['output_prefix'] + '.yaml').is_file())
                self.assertEqual(response.json()['map']['name'], 'floor_1')
                self.assertEqual(response.json()['map']['slam_session_state']['status'], 'AVAILABLE')
                self.assertFalse(response.json()['map']['canonical_map_promoted'])

    def test_map_save_requires_stopped_mapping_and_current_session_map(self):
        runtime.robot_mapping_state['R01'] = 'MAPPING'
        with patch('twin.local_control_views._bridge_request') as request:
            active = self.client.post('/api/robots/R01/local/maps/save',
                                      data=json.dumps({'name': 'moving'}), content_type='application/json')
        self.assertEqual(active.status_code, 409)
        self.assertIn('stop mapping before saving', active.json()['detail'].lower())
        request.assert_not_called()

        runtime.robot_mapping_state['R01'] = 'PAUSED'
        runtime.robot_map_snapshots['R01'] = {'map': {
            'robot_id': 'R01', 'map_source': 'NAV2_MAP',
            'mapping_session_id': 'stale-session', 'known_cells': 10,
        }}
        with patch('twin.local_control_views._bridge_request') as request:
            stale = self.client.post('/api/robots/R01/local/maps/save',
                                     data=json.dumps({'name': 'stale'}), content_type='application/json')
        self.assertEqual(stale.status_code, 409)
        self.assertIn('current slam toolbox session', stale.json()['detail'].lower())
        request.assert_not_called()

    def test_load_map_and_initial_pose_route_to_the_robot_bridge(self):
        self.old_operation_mode = runtime.operation_mode
        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            prefix = map_output_prefix('R01', 'floor')
            prefix.with_suffix('.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes(10_000))
            prefix.with_suffix('.yaml').write_text(
                'image: floor.pgm\nresolution: 0.1\norigin: [-5, -5, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
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

    def test_one_saved_map_load_stops_slam_switches_runtime_and_confirms_local_map(self):
        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            prefix = map_output_prefix('R01', 'floor')
            prefix.with_suffix('.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes(10_000))
            prefix.with_suffix('.yaml').write_text(
                'image: floor.pgm\nresolution: 0.1\norigin: [-5, -5, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
            map_record = register_saved_map('R01', 'floor', prefix.with_suffix('.yaml'))
            runtime.operation_mode = 'MAPPING'
            runtime.robot_mapping_state['R01'] = 'MAPPING'

            with patch.object(SimulationRuntimeAdapter, 'get_mode_status', return_value={
                    'robot_id': 'R01', 'mode': 'mapping', 'status': 'READY'}), \
                    patch.object(SimulationRuntimeAdapter, 'request_mode_change', return_value={
                        'ok': True, 'robot_id': 'R01', 'mode': 'navigation',
                        'request_id': 'nav-transition-1', 'status': 'REQUESTED',
                        'message': 'supervised transition queued'}), \
                    patch('twin.local_control_views._bridge_request', return_value={
                        'ok': True, 'result': {'mapping_state': 'PAUSED'}
                    }) as bridge:
                queued = self.client.post(
                    '/api/robots/R01/local/maps/load',
                    data=json.dumps({'map_id': map_record['id']}), content_type='application/json')
            self.assertEqual(queued.status_code, 202, queued.content)
            self.assertEqual(queued.json()['status'], 'TRANSITIONING')
            self.assertEqual(queued.json()['request_id'], 'nav-transition-1')
            self.assertEqual(bridge.call_args.args[1], 'MAPPING_STOP')
            self.assertIn('R01', runtime.local_map_transitions)
            self.assertEqual(runtime.pending_local_map_loads['R01']['map_id'], map_record['id'])

            runtime.operation_mode = 'NAVIGATION'
            with patch.object(SimulationRuntimeAdapter, 'get_mode_status', return_value={
                    'robot_id': 'R01', 'mode': 'navigation', 'status': 'READY',
                    'request_id': 'nav-transition-1'}), \
                    patch('twin.local_control_views._bridge_request', return_value={
                        'ok': True, 'result': {
                            'map_id': map_record['id'],
                            'active_map_revision': map_record['revision'],
                        },
                    }) as bridge:
                loaded = self.client.post(
                    '/api/robots/R01/local/maps/load',
                    data=json.dumps({'map_id': map_record['id']}), content_type='application/json')
            self.assertEqual(loaded.status_code, 200, loaded.content)
            self.assertEqual(bridge.call_args.args[1], 'MAP_LOAD')
            self.assertEqual(bridge.call_args.args[2]['map_yaml'], str(prefix.with_suffix('.yaml')))
            self.assertEqual(loaded.json()['map_sync_status'], 'LOCAL_ONLY')
            self.assertEqual(loaded.json()['map_source'], 'SAVED_LOCAL')
            self.assertEqual(runtime.active_map_state('R01')['local_active_map_id'], map_record['id'])
            self.assertNotIn('R01', runtime.local_map_transitions)
            self.assertNotIn('R01', runtime.pending_local_map_loads)

    def test_invalid_local_map_is_rejected_before_stopping_slam_or_switching_modes(self):
        with patch.object(SimulationRuntimeAdapter, 'request_mode_change') as switch, \
                patch('twin.local_control_views._bridge_request') as bridge:
            response = self.client.post(
                '/api/robots/R01/local/maps/load',
                data=json.dumps({'map_id': 'not-registered'}), content_type='application/json')
        self.assertEqual(response.status_code, 404)
        switch.assert_not_called()
        bridge.assert_not_called()

    def test_saved_slam_resume_restarts_mapping_then_requires_live_old_map_match(self):
        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            prefix = map_output_prefix('R01', 'resume')
            prefix.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 254, 205, 254]))
            prefix.with_suffix('.yaml').write_text(
                'image: resume.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
            session_prefix = prefix.parent / 'resume_slam_session'
            session_prefix.with_suffix('.posegraph').write_bytes(b'pose graph')
            session_prefix.with_suffix('.data').write_bytes(b'sensor data')
            record = register_saved_map('R01', 'resume', prefix.with_suffix('.yaml'), slam_session={
                'engine': 'SLAM_TOOLBOX', 'status': 'AVAILABLE',
                'posegraph_path': str(session_prefix.with_suffix('.posegraph')),
                'data_path': str(session_prefix.with_suffix('.data')),
            })
            runtime.operation_mode = 'NAVIGATION'
            runtime.local_map_overrides['R01'] = record['id']
            runtime.local_map_revisions['R01'] = record['revision']
            transition_states = [
                {'robot_id': 'R01', 'mode': 'navigation', 'status': 'READY'},
                {'robot_id': 'R01', 'mode': 'mapping', 'status': 'RESTARTING', 'request_id': 'resume-1'},
                {'robot_id': 'R01', 'mode': 'mapping', 'status': 'READY', 'request_id': 'resume-1'},
            ]
            with patch.object(SimulationRuntimeAdapter, 'get_mode_status', side_effect=transition_states), \
                    patch.object(SimulationRuntimeAdapter, 'request_mode_change', return_value={
                        'ok': True, 'robot_id': 'R01', 'mode': 'mapping',
                        'request_id': 'resume-1', 'status': 'REQUESTED',
                        'message': 'restart requested',
                        'slam_session_file': str(session_prefix)} ) as switch, \
                    patch('twin.local_control_views._robot_available', side_effect=[
                        None, JsonResponse({'ok': False, 'error': 'bridge is offline'}, status=503),
                    ]) as availability:
                queued = self.client.post('/api/robots/R01/local/maps/resume-session',
                    data=json.dumps({'map_id': record['id']}), content_type='application/json')
                self.assertEqual(queued.status_code, 202, queued.content)
                self.assertEqual(queued.json()['status'], 'TRANSITIONING')
                self.assertNotIn(str(session_prefix), queued.content.decode('utf-8'))
                self.assertEqual(switch.call_args.kwargs['slam_session_file'], str(session_prefix))

                # The bridge is intentionally offline during its supervised
                # process restart. Polling the already-pending transition
                # must not be rejected by the initial online-bridge guard.
                transition = self.client.post('/api/robots/R01/local/maps/resume-session',
                    data=json.dumps({'map_id': record['id']}), content_type='application/json')
                self.assertEqual(transition.status_code, 202, transition.content)
                self.assertEqual(transition.json()['transition']['status'], 'RESTARTING')
                self.assertEqual(availability.call_count, 1)

                encoded = base64.b64encode(zlib.compress(bytes([0, 1, 101, 1]))).decode('ascii')
                runtime.operation_mode = 'MAPPING'
                runtime.robot_map_snapshots['R01'] = {'map': {
                    'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'resumed-runtime',
                    'map_version': 1, 'timestamp': '2026-10-02T00:00:00Z', 'stamp': 2.0,
                    'width': 2, 'height': 2, 'resolution': 0.1, 'known_cells': 3,
                    'origin': {'x': 0, 'y': 0, 'yaw': 0},
                    'data_encoding': 'zlib-base64-offset1', 'data_zlib_base64': encoded,
                }}
                resumed = self.client.post('/api/robots/R01/local/maps/resume-session',
                    data=json.dumps({'map_id': record['id']}), content_type='application/json')
            self.assertEqual(resumed.status_code, 200, resumed.content)
            self.assertEqual(resumed.json()['status'], 'RESUMED')
            self.assertTrue(resumed.json()['restore_evidence']['passed'])
            self.assertNotIn('R01', runtime.local_map_overrides)
            self.assertEqual(runtime.robot_mapping_state['R01'], 'MAPPING')
            self.assertNotIn('R01', runtime.pending_slam_session_resumes)
            self.assertNotIn('R01', runtime.local_map_transitions)

    def test_failed_local_map_load_clears_pending_transition_state(self):
        with tempfile.TemporaryDirectory() as tempdir, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tempdir)):
            prefix = map_output_prefix('R01', 'floor')
            prefix.with_suffix('.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes(10_000))
            prefix.with_suffix('.yaml').write_text(
                'image: floor.pgm\nresolution: 0.1\norigin: [-5, -5, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
            map_record = register_saved_map('R01', 'floor', prefix.with_suffix('.yaml'))
            runtime.operation_mode = 'NAVIGATION'
            failed_results = (
                (TimeoutError('bridge timeout'), 502),
                ({'ok': False, 'error': 'map_server rejected the saved artifact'}, 502),
                ({'ok': True, 'result': {
                    'map_id': 'different-map', 'active_map_revision': 'different-revision',
                }}, 502),
            )
            with patch('twin.local_control_views._bridge_request') as bridge:
                for result, expected_status in failed_results:
                    with self.subTest(result=result):
                        bridge.side_effect = result if isinstance(result, Exception) else None
                        if not isinstance(result, Exception):
                            bridge.return_value = result
                        response = self.client.post(
                            '/api/robots/R01/local/maps/load',
                            data=json.dumps({'map_id': map_record['id']}),
                            content_type='application/json')
                        self.assertEqual(response.status_code, expected_status, response.content)
                        self.assertNotIn('R01', runtime.local_map_transitions)
                        self.assertNotIn('R01', runtime.pending_local_map_loads)

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

    def test_initial_pose_rejects_nonfinite_malformed_and_out_of_range_values(self):
        robot = runtime.engine.state['robots']['R01']
        robot.update({'control_mode': 'MANUAL', 'navigation_state': 'IDLE',
                      'vx': 0.0, 'vy': 0.0, 'wz': 0.0})
        invalid_poses = (
            {'x': float('nan'), 'y': 0.0, 'yaw': 0.0},
            {'x': 0.0, 'y': 0.0, 'yaw': float('inf')},
            {'x': 0.0, 'y': 0.0, 'yaw': 'north'},
            {'x': 0.0, 'y': 0.0, 'yaw': 3.2},
        )
        with patch('twin.local_control_views._bridge_request') as bridge:
            for pose in invalid_poses:
                with self.subTest(pose=pose):
                    response = self.client.post(
                        '/api/robots/R01/local/initial-pose',
                        data=json.dumps({**pose, 'frame_id': 'map'}),
                        content_type='application/json')
                    self.assertEqual(response.status_code, 400, response.content)
            bridge.assert_not_called()

    def test_initial_pose_is_blocked_during_local_map_transition(self):
        robot = runtime.engine.state['robots']['R01']
        robot.update({'control_mode': 'MANUAL', 'navigation_state': 'IDLE',
                      'vx': 0.0, 'vy': 0.0, 'wz': 0.0})
        runtime.local_map_transitions.add('R01')
        try:
            with patch('twin.local_control_views._bridge_request') as bridge:
                response = self.client.post(
                    '/api/robots/R01/local/initial-pose',
                    data=json.dumps({'x': 2.0, 'y': 3.0, 'yaw': 1.2, 'frame_id': 'map'}),
                    content_type='application/json')
            self.assertEqual(response.status_code, 409)
            bridge.assert_not_called()
        finally:
            runtime.local_map_transitions.discard('R01')

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

    def test_saved_slam_mode_queue_keeps_session_prefix_out_of_public_status(self):
        session_prefix = '/private/local_robot_maps/R01/session'
        with tempfile.TemporaryDirectory() as tempdir, override_settings(
                WARETWIN_STACK_RUNTIME_DIR=Path(tempdir)):
            runtime_dir = Path(tempdir)
            status_file = runtime_dir / 'mode-switch-status.json'
            status_file.write_text(json.dumps({
                'robot_id': 'R01', 'mode': 'navigation', 'status': 'READY',
            }), encoding='utf-8')
            queued = SimulationRuntimeAdapter().request_mode_change(
                'R01', 'mapping', slam_session_file=session_prefix)
            self.assertTrue(queued['ok'])
            self.assertNotIn(session_prefix, json.dumps(queued))
            self.assertNotIn(session_prefix, status_file.read_text(encoding='utf-8'))
            request = json.loads((runtime_dir / 'mode-switch-request.json').read_text(encoding='utf-8'))
            self.assertEqual(request['slam_session_file'], session_prefix)

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
