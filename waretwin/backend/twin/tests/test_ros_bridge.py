from copy import deepcopy
import threading
import time
from types import SimpleNamespace
from unittest import IsolatedAsyncioTestCase
from unittest.mock import AsyncMock, patch

from pydantic import TypeAdapter

from twin.coordinates import ros_pose_to_waretwin, ros_twist_to_waretwin
from twin.gateways.ros_bridge import RosBridgeGateway
from twin.ros_bridge_consumer import RosBridgeConsumer, RosBridgeRegistry, registry
from twin.runtime import runtime
from twin.schema import ClientMessage


class RosCoordinateTests(IsolatedAsyncioTestCase):
    async def test_detail_view_request_preserves_correlation_and_receive_timing(self):
        consumer = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'robot_bridge_online', return_value=True), \
                patch.object(runtime, 'gateway', return_value=gateway):
            await runtime.handle_message(consumer, {'type': 'ROBOT_DETAIL_VIEW',
                'robot_id': 'R01', 'view': 'LIDAR_3D', 'request_id': 'view-new',
                '_view_received_ms': 12345.0})
        gateway.send_command.assert_awaited_once_with('R01', 'DETAIL_VIEW', {
            'view': 'LIDAR_3D', 'request_id': 'view-new', 'django_received_ms': 12345.0})
        consumer.send_json.assert_not_awaited()

    async def test_detail_view_ack_is_broadcast_without_waiting_for_sensor_frame(self):
        status = {'type': 'ROBOT_DETAIL_VIEW_STATUS', 'robot_id': 'R01',
            'requested_view': 'LIDAR_3D', 'applied_view': 'LIDAR_3D',
            'view_epoch': 2, 'request_id': 'view-new', 'state': 'APPLIED'}
        with patch.object(runtime, 'broadcast', new_callable=AsyncMock) as broadcast:
            await runtime.handle_ros_message(status)
        broadcast.assert_awaited_once_with(status)
        self.assertIn('ROBOT_DETAIL_VIEW_STATUS', RosBridgeConsumer.database_free_types)

    async def test_latest_valid_map_snapshot_is_retained_for_new_control_clients(self):
        previous = runtime.robot_map_snapshots.copy()
        previous_runtime = runtime.robot_runtime_map_snapshots.copy()
        previous_geometry = runtime.robot_map_geometry.copy()
        payload = {
            'type': 'MAP_SNAPSHOT',
            'map': {
                'robot_id': 'R01', 'frame_id': 'map', 'width': 2, 'height': 2,
                'resolution': 0.05, 'origin': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
                'data': [-1, 0, 100, 50], 'active_map_id': 'CANONICAL',
                'active_map_revision': '21',
            },
        }
        try:
            runtime.robot_map_snapshots.clear()
            with patch.object(runtime, 'broadcast', new_callable=AsyncMock) as broadcast:
                await runtime.handle_ros_message(payload)
            self.assertEqual(runtime.robot_map_snapshots['R01'], payload)
            self.assertEqual(runtime.robot_runtime_map_snapshots['R01'], payload)
            broadcast.assert_awaited_once_with(payload)
        finally:
            runtime.robot_map_snapshots.clear()
            runtime.robot_map_snapshots.update(previous)
            runtime.robot_runtime_map_snapshots.clear()
            runtime.robot_runtime_map_snapshots.update(previous_runtime)
            runtime.robot_map_geometry.clear()
            runtime.robot_map_geometry.update(previous_geometry)

    async def test_slam_and_runtime_map_snapshots_are_cached_independently(self):
        previous = runtime.robot_map_snapshots.copy()
        previous_runtime = runtime.robot_runtime_map_snapshots.copy()
        previous_slam = runtime.robot_slam_map_snapshots.copy()
        previous_geometry = runtime.robot_map_geometry.copy()
        runtime_snapshot = {'type': 'MAP_SNAPSHOT', 'map': {
            'robot_id': 'R01', 'frame_id': 'map', 'map_source': 'NAV2_MAP',
            'active_map_id': 'CANONICAL', 'active_map_revision': '21',
            'width': 1, 'height': 1, 'resolution': 0.05,
            'origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'data': [0],
        }}
        slam_snapshot = {'type': 'MAP_SNAPSHOT', 'map': {
            'robot_id': 'R01', 'frame_id': 'map', 'map_source': 'SLAM_TOOLBOX',
            'mapping_session_id': 'session-1', 'active_map_id': 'SLAM-session-1',
            'active_map_revision': 'slam-r1', 'width': 1, 'height': 1,
            'resolution': 0.05, 'origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'data': [100],
        }}
        try:
            for cache in (runtime.robot_map_snapshots, runtime.robot_runtime_map_snapshots,
                          runtime.robot_slam_map_snapshots):
                cache.pop('R01', None)
            with patch.object(runtime, 'broadcast', new_callable=AsyncMock):
                await runtime.handle_ros_message(runtime_snapshot)
                await runtime.handle_ros_message(slam_snapshot)
            self.assertEqual(runtime.robot_runtime_map_snapshots['R01'], runtime_snapshot)
            self.assertEqual(runtime.robot_slam_map_snapshots['R01'], slam_snapshot)
            self.assertEqual(runtime.robot_map_snapshots['R01'], slam_snapshot)
        finally:
            runtime.robot_map_snapshots.clear(); runtime.robot_map_snapshots.update(previous)
            runtime.robot_runtime_map_snapshots.clear(); runtime.robot_runtime_map_snapshots.update(previous_runtime)
            runtime.robot_slam_map_snapshots.clear(); runtime.robot_slam_map_snapshots.update(previous_slam)
            runtime.robot_map_geometry.clear(); runtime.robot_map_geometry.update(previous_geometry)

    async def test_mapping_health_diagnostics_are_retained_for_mapping_ui(self):
        previous = runtime.ros_diagnostics.get('mapping')
        measured = {
            'slam_state': 'ACTIVE', 'scan_live': True, 'scan_hz': 8.5,
            'scan_frame': 'lidar_link', 'odom_live': True, 'odom_hz': 42.0,
            'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'tf_valid': True, 'map_live': True, 'map_odom_owner': 'SLAM_TOOLBOX',
        }
        try:
            with patch.object(runtime, 'broadcast_runtime_status', new_callable=AsyncMock):
                await runtime.handle_ros_diagnostics({'diagnostics': {'mapping': measured}})
            self.assertEqual(runtime.ros_diagnostics['mapping'], measured)
        finally:
            if previous is None:
                runtime.ros_diagnostics.pop('mapping', None)
            else:
                runtime.ros_diagnostics['mapping'] = previous

    async def test_runtime_status_exposes_robot_mapping_session_identity(self):
        old_connected = set(runtime.connected_robot_ids)
        old_sessions = dict(runtime.robot_mapping_sessions)
        try:
            runtime.connected_robot_ids.add('R01')
            runtime.robot_mapping_sessions['R01'] = 'session-current'
            status = runtime.runtime_status_message()
            self.assertEqual(status['robot_mapping_sessions']['R01'], 'session-current')
        finally:
            runtime.connected_robot_ids = old_connected
            runtime.robot_mapping_sessions.clear()
            runtime.robot_mapping_sessions.update(old_sessions)

    async def test_mapping_heartbeat_invalidates_old_map_but_keeps_same_session_snapshot(self):
        snapshots = runtime.robot_map_snapshots.copy()
        slam_snapshots = runtime.robot_slam_map_snapshots.copy()
        geometry = runtime.robot_map_geometry.copy()
        modes = runtime.robot_runtime_modes.copy()
        sessions = runtime.robot_mapping_sessions.copy()
        old_mode = runtime.operation_mode
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        try:
            runtime.operation_mode = 'NAVIGATION'
            runtime.connected_robot_ids.add('R01')
            runtime.robot_runtime_modes['R01'] = 'NAVIGATION'
            runtime.robot_map_snapshots['R01'] = {'map': {'map_source': 'NAV2_MAP'}}
            runtime.robot_slam_map_snapshots.pop('R01', None)
            runtime.robot_map_geometry['R01'] = {'width': 3}
            with patch.object(runtime, 'broadcast_runtime_status', new_callable=AsyncMock):
                await runtime.handle_ros_message({
                    'type': 'HEARTBEAT', 'robot_id': 'R01', 'runtime_state': 'MAPPING',
                    'mapping_state': 'MAPPING', 'mapping_session_id': 'session-1',
                })
            self.assertNotIn('R01', runtime.robot_map_snapshots)
            self.assertNotIn('R01', runtime.robot_slam_map_snapshots)
            self.assertNotIn('R01', runtime.robot_map_geometry)

            current = {'map': {'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'session-1'}}
            runtime.robot_map_snapshots['R01'] = current
            runtime.robot_slam_map_snapshots['R01'] = current
            with patch.object(runtime, 'broadcast_runtime_status', new_callable=AsyncMock):
                await runtime.handle_ros_message({
                    'type': 'HEARTBEAT', 'robot_id': 'R01', 'runtime_state': 'MAPPING',
                    'mapping_state': 'MAPPING', 'mapping_session_id': 'session-1',
                })
            self.assertIs(runtime.robot_map_snapshots['R01'], current)
            self.assertIs(runtime.robot_slam_map_snapshots['R01'], current)
        finally:
            runtime.robot_map_snapshots.clear(); runtime.robot_map_snapshots.update(snapshots)
            runtime.robot_slam_map_snapshots.clear(); runtime.robot_slam_map_snapshots.update(slam_snapshots)
            runtime.robot_map_geometry.clear(); runtime.robot_map_geometry.update(geometry)
            runtime.robot_runtime_modes.clear(); runtime.robot_runtime_modes.update(modes)
            runtime.robot_mapping_sessions.clear(); runtime.robot_mapping_sessions.update(sessions)
            runtime.operation_mode = old_mode
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats

    async def test_ros_pose_uses_single_waretwin_adapter(self):
        pose = ros_pose_to_waretwin(1.2, 3.4, 0.5, 1.57)
        self.assertEqual(pose['position'], [1.2, 0.5, 3.4])
        self.assertAlmostEqual(pose['heading'], 1.57, places=2)

    async def test_twist_keeps_holonomic_components(self):
        twist = ros_twist_to_waretwin(0.4, 0.3, -0.2)
        self.assertEqual(twist['vx'], 0.4)
        self.assertEqual(twist['vy'], 0.3)
        self.assertEqual(twist['wz'], -0.2)
        self.assertAlmostEqual(twist['velocity'], 0.5)

    async def test_robot_scoped_bridge_rpc_is_resolved_thread_safely(self):
        local_registry = RosBridgeRegistry()

        class Capture:
            robot_id = 'R01'

            async def send_json(self, payload):
                response = {
                    'type': 'LOCAL_CONTROL_RESULT', 'robot_id': 'R01',
                    'request_id': payload['request_id'], 'operation': 'MAPPING_START',
                    'ok': True, 'result': {'mapping_state': 'MAPPING'},
                }
                worker = threading.Thread(target=local_registry.resolve_request, args=(response,))
                worker.start()
                worker.join()

        local_registry.consumers['R01'] = Capture()
        result = await local_registry.request({
            'type': 'LOCAL_CONTROL', 'robot_id': 'R01', 'request_id': 'rpc-1',
            'operation': 'MAPPING_START',
        }, timeout=1.0)
        self.assertTrue(result['ok'])
        self.assertEqual(result['robot_id'], 'R01')
        self.assertEqual(result['result']['mapping_state'], 'MAPPING')

    async def test_path_preview_is_robot_scoped_and_send_goal_requires_matching_preview(self):
        old_values = {
            'runtime_mode': runtime.runtime_mode,
            'operation_mode': runtime.operation_mode,
            'published_map_revision': runtime.published_map_revision,
            'robot_map_sync': deepcopy(runtime.robot_map_sync),
            'connected_robot_ids': set(runtime.connected_robot_ids),
            'path_preview_requests': dict(runtime.path_preview_requests),
            'approved_path_previews': deepcopy(runtime.approved_path_previews),
            'path_preview_results': deepcopy(runtime.path_preview_results),
            'expired_path_previews': dict(runtime.expired_path_previews),
            'path_preview_invalidations': dict(runtime.path_preview_invalidations),
            'robots': deepcopy(runtime.engine.state.get('robots', {})),
            'robot_pose_heartbeats': dict(runtime.robot_pose_heartbeats),
        }
        runtime.runtime_mode = 'GAZEBO_ROS'
        runtime.operation_mode = 'NAVIGATION'
        runtime.published_map_revision = 21
        runtime.connected_robot_ids.update(('R01', 'R02'))
        runtime.robot_map_sync.update({
            rid: {'ros_revision': 21, 'status': 'SYNCED', 'tf_status': True,
                  'gazebo_revision': 21, 'nav2_revision': 21, 'tag_map_revision': 21}
            for rid in ('R01', 'R02')
        })
        runtime.path_preview_requests.clear()
        runtime.approved_path_previews.clear()
        runtime.path_preview_results.clear()
        runtime.expired_path_previews.clear()
        runtime.path_preview_invalidations.clear()
        runtime.robot_pose_heartbeats['R01'] = time.monotonic()
        runtime.engine.state.setdefault('robots', {}).setdefault('R01', {})['active_map_pose'] = {
            'valid': True, 'frame_id': 'map', 'map_id': 'CANONICAL',
            'map_revision': '21', 'map_source': 'CANONICAL', 'pose_source': 'TF',
        }
        capture = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))

        async def request_preview(request_id, *, result_status='VALID'):
            await runtime.handle_message(capture, {
                'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01', 'request_id': request_id,
                'x': 2.0, 'y': 3.0, 'yaw': 0.4, 'frame_id': 'map',
                'active_map_id': 'CANONICAL', 'active_map_revision': '21',
            })
            await runtime.handle_ros_message({
                'type': 'PATH_PREVIEW_RESULT', 'robot_id': 'R01', 'request_id': request_id,
                'status': result_status,
                'path': [[0.0, 0.0], [2.0, 3.0]] if result_status == 'VALID' else [],
                'goal': {'x': 2.0, 'y': 3.0, 'yaw': 0.4},
                'active_map_id': 'CANONICAL', 'active_map_revision': '21',
            })

        async def send_goal(request_id, *, robot_id='R01', x=2.0, map_id='CANONICAL', revision='21'):
            return await runtime.handle_message(capture, {
                'type': 'NAV_GOAL', 'robot_id': robot_id, 'x': x, 'y': 3.0, 'yaw': 0.4,
                'frame_id': 'map', 'preview_request_id': request_id,
                'active_map_id': map_id, 'active_map_revision': revision,
            })

        def navigation_calls():
            return [call for call in gateway.send_command.await_args_list
                    if len(call.args) > 1 and call.args[1] == 'NAVIGATE']

        try:
            with patch.object(runtime, 'robot_bridge_online', return_value=True), \
                    patch.object(runtime, 'gateway', return_value=gateway), \
                    patch.object(runtime, 'broadcast', new=AsyncMock()):
                await request_preview('preview-valid')
                gateway.send_command.assert_awaited_with('R01', 'PATH_PREVIEW', {
                    'request_id': 'preview-valid', 'x': 2.0, 'y': 3.0, 'yaw': 0.4,
                    'source_type': 'MAP_POINT', 'source_id': None, 'tag_id': None,
                    'tag_revision': None, 'registry_revision': None,
                    'frame_id': 'map', 'active_map_id': 'CANONICAL',
                    'active_map_revision': '21', 'canonical_map_revision': 21,
                })
                await send_goal('preview-valid', robot_id='R02')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
                self.assertEqual(navigation_calls(), [])

                capture.send_json.reset_mock()
                await send_goal('preview-valid', x=2.5)
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
                self.assertEqual(navigation_calls(), [])

                capture.send_json.reset_mock()
                await send_goal('preview-valid', revision='20')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_MAP_MISMATCH')
                self.assertEqual(navigation_calls(), [])

                runtime.published_map_revision = 22
                runtime.robot_map_sync['R01'] = {
                    **runtime.robot_map_sync['R01'], 'ros_revision': 22,
                    'gazebo_revision': 22, 'nav2_revision': 22, 'tag_map_revision': 22,
                }
                capture.send_json.reset_mock()
                await send_goal('preview-valid', revision='22')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_MAP_MISMATCH')
                self.assertEqual(navigation_calls(), [])
                runtime.published_map_revision = 21
                runtime.robot_map_sync['R01'] = {
                    **runtime.robot_map_sync['R01'], 'ros_revision': 21,
                    'gazebo_revision': 21, 'nav2_revision': 21, 'tag_map_revision': 21,
                }

                runtime.robot_pose_heartbeats['R01'] = time.monotonic() - 10.0
                await runtime.handle_message(capture, {
                    'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01',
                    'request_id': 'preview-stale-localization',
                    'x': 2.0, 'y': 3.0, 'yaw': 0.4, 'frame_id': 'map',
                    'active_map_id': 'CANONICAL', 'active_map_revision': '21',
                })
                self.assertEqual(capture.send_json.await_args.args[0]['type'], 'PATH_PREVIEW_RESULT')
                self.assertEqual(capture.send_json.await_args.args[0]['status'], 'INVALID')
                self.assertIn('fresh TF pose', capture.send_json.await_args.args[0]['reason'])
                runtime.robot_pose_heartbeats['R01'] = time.monotonic()

                await request_preview('preview-localization-change')
                runtime.engine.state['robots']['R01']['active_map_pose']['pose_source'] = 'GAZEBO_MODEL_STATES'
                capture.send_json.reset_mock()
                await send_goal('preview-localization-change')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_MAP_MISMATCH')
                self.assertEqual(navigation_calls(), [])
                runtime.engine.state['robots']['R01']['active_map_pose']['pose_source'] = 'TF'

                key = ('R01', 'preview-valid')
                runtime.path_preview_results[key]['created_monotonic'] = time.monotonic() - 121
                runtime.approved_path_previews[key]['created_monotonic'] = time.monotonic() - 121
                capture.send_json.reset_mock()
                await send_goal('preview-valid')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_EXPIRED')
                self.assertEqual(navigation_calls(), [])

                await request_preview('preview-no-path', result_status='NO_PATH')
                capture.send_json.reset_mock()
                await send_goal('preview-no-path')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'NO_VALID_PATH')
                self.assertEqual(navigation_calls(), [])

                capture.send_json.reset_mock()
                await runtime.handle_message(capture, {
                    'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 2.0, 'y': 3.0, 'yaw': 0.4,
                    'frame_id': 'map', 'active_map_id': 'CANONICAL', 'active_map_revision': '21',
                })
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_REQUIRED')
                self.assertEqual(navigation_calls(), [])

                await request_preview('preview-approved')
                await runtime.handle_message(capture, {
                    'type': 'PATH_PREVIEW_INVALIDATE', 'robot_id': 'R01',
                })
                self.assertNotIn(('R01', 'preview-approved'), runtime.path_preview_results)
                self.assertNotIn(('R01', 'preview-approved'), runtime.approved_path_previews)
                capture.send_json.reset_mock()
                await send_goal('preview-approved')
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
                self.assertEqual(navigation_calls(), [])

                await request_preview('preview-approved')
                await send_goal('preview-approved')
                self.assertEqual(len(navigation_calls()), 1)
                self.assertEqual(gateway.send_command.await_args.args, ('R01', 'NAVIGATE', {
                    'x': 2.0, 'y': 3.0, 'yaw': 0.4, 'frame_id': 'map',
                    'source_type': 'MAP_POINT', 'source_id': None, 'tag_id': None,
                    'tag_revision': None, 'registry_revision': None,
                    'preview_request_id': 'preview-approved',
                    'active_map_id': 'CANONICAL', 'active_map_revision': '21',
                    'canonical_map_revision': 21,
                }))
        finally:
            for key, value in old_values.items():
                if key == 'robots':
                    runtime.engine.state['robots'] = value
                else:
                    setattr(runtime, key, value)

    async def test_local_map_point_preview_and_send_stay_on_exact_active_identity(self):
        active = {'value': {'active_map_id': 'saved-R01-1', 'active_map_revision': 'artifact-1',
            'canonical_map_revision': 21, 'map_sync_status': 'LOCAL_ONLY'}}
        capture = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))

        def request_preview(request_id, map_id, revision):
            return {'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01', 'request_id': request_id,
                'x': 1.25, 'y': -0.5, 'yaw': 0.3, 'frame_id': 'map',
                'map_id': map_id, 'map_revision': revision,
                'active_map_id': map_id, 'active_map_revision': revision}

        def planner_result(request_id, map_id, revision):
            return {'type': 'PATH_PREVIEW_RESULT', 'robot_id': 'R01', 'request_id': request_id,
                'status': 'VALID', 'path': [[0.0, 0.0], [1.25, -0.5]],
                'goal': {'x': 1.25, 'y': -0.5, 'yaw': 0.3},
                'active_map_id': map_id, 'active_map_revision': revision}

        def send_goal(request_id, map_id, revision):
            return {'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 1.25, 'y': -0.5, 'yaw': 0.3,
                'frame_id': 'map', 'preview_request_id': request_id,
                'map_id': map_id, 'map_revision': revision,
                'active_map_id': map_id, 'active_map_revision': revision}

        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'active_map_state', side_effect=lambda _rid: active['value']), \
                patch.object(runtime, 'robot_bridge_online', return_value=True), \
                patch.object(runtime, 'navigation_localization_state', side_effect=lambda _rid, state: state['active_map_id']), \
                patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime, 'broadcast', new=AsyncMock()), \
                patch.object(runtime, 'path_preview_requests', {}), \
                patch.object(runtime, 'approved_path_previews', {}), \
                patch.object(runtime, 'path_preview_results', {}), \
                patch.object(runtime, 'expired_path_previews', {}), \
                patch.object(runtime, 'path_preview_invalidations', {}):
            wrong_map = request_preview('local-wrong-map', 'saved-R01-1', 'artifact-1')
            wrong_map['map_id'] = 'CANONICAL'
            await runtime.handle_message(capture, wrong_map)
            self.assertEqual(capture.send_json.await_args.args[0]['status'], 'INVALID')
            self.assertIn('PATH_PREVIEW_MAP_MISMATCH', capture.send_json.await_args.args[0]['reason'])
            self.assertFalse(any(call.args[1] == 'PATH_PREVIEW' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            wrong_revision = request_preview('local-wrong-revision', 'saved-R01-1', 'artifact-1')
            wrong_revision['map_revision'] = 'artifact-other'
            await runtime.handle_message(capture, wrong_revision)
            self.assertEqual(capture.send_json.await_args.args[0]['status'], 'INVALID')
            self.assertIn('PATH_PREVIEW_MAP_MISMATCH', capture.send_json.await_args.args[0]['reason'])
            self.assertFalse(any(call.args[1] == 'PATH_PREVIEW' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            await runtime.handle_message(capture, request_preview('local-old', 'saved-R01-1', 'artifact-1'))
            self.assertEqual(gateway.send_command.await_args.args, ('R01', 'PATH_PREVIEW', {
                'request_id': 'local-old', 'x': 1.25, 'y': -0.5, 'yaw': 0.3,
                'source_type': 'MAP_POINT', 'source_id': None, 'tag_id': None,
                'tag_revision': None, 'registry_revision': None, 'frame_id': 'map',
                'active_map_id': 'saved-R01-1', 'active_map_revision': 'artifact-1',
                'canonical_map_revision': 21,
            }))
            await runtime.handle_ros_message(planner_result('local-old', 'saved-R01-1', 'artifact-1'))
            self.assertEqual(runtime.path_preview_results[('R01', 'local-old')]['status'], 'VALID')

            active['value'] = {**active['value'], 'active_map_id': 'saved-R01-2',
                'active_map_revision': 'artifact-2'}
            await runtime.handle_message(capture, send_goal('local-old', 'saved-R01-1', 'artifact-1'))
            self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_MAP_MISMATCH')
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            await runtime.handle_message(capture, request_preview('local-current', 'saved-R01-2', 'artifact-2'))
            await runtime.handle_ros_message(planner_result('local-current', 'saved-R01-2', 'artifact-2'))
            await runtime.handle_message(capture, send_goal('local-current', 'saved-R01-2', 'artifact-2'))
            self.assertEqual(gateway.send_command.await_args.args, ('R01', 'NAVIGATE', {
                'x': 1.25, 'y': -0.5, 'yaw': 0.3, 'frame_id': 'map',
                'source_type': 'MAP_POINT', 'source_id': None, 'tag_id': None,
                'tag_revision': None, 'registry_revision': None,
                'preview_request_id': 'local-current', 'active_map_id': 'saved-R01-2',
                'active_map_revision': 'artifact-2', 'canonical_map_revision': 21,
            }))
            self.assertEqual(sum(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list), 1)

    async def test_gateway_does_not_publish_without_bridge(self):
        previous = registry.consumer
        registry.consumer = None
        try:
            result = await RosBridgeGateway().send_command('R01', 'NAVIGATE', {'x': 1, 'y': 2, 'yaw': 0})
            self.assertFalse(result['ok'])
            self.assertEqual(result['message']['type'], 'NAV_GOAL')
        finally:
            registry.consumer = previous

    async def test_local_map_transition_blocks_autonomous_mode_and_manual_motion(self):
        previous_transitions = runtime.local_map_transitions.copy()
        runtime.local_map_transitions.add('R01')
        capture = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        try:
            with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                    patch.object(runtime, 'robot_bridge_online', return_value=True), \
                    patch.object(runtime, 'gateway', return_value=gateway):
                await runtime.handle_message(capture, {
                    'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'AUTONOMOUS',
                })
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'LOCAL_MAP_TRANSITION')
                capture.send_json.reset_mock()

                await runtime.handle_message(capture, {
                    'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'FORWARD',
                })
                self.assertEqual(capture.send_json.await_args.args[0]['code'], 'LOCAL_MAP_TRANSITION')
                gateway.send_command.assert_not_awaited()

                capture.send_json.reset_mock()
                await runtime.handle_message(capture, {
                    'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP',
                })
                gateway.send_command.assert_awaited_once_with('R01', 'MANUAL_CMD', {'action': 'STOP'})
        finally:
            runtime.local_map_transitions.clear()
            runtime.local_map_transitions.update(previous_transitions)

    async def test_manual_control_messages_are_schema_validated(self):
        mode = TypeAdapter(ClientMessage).validate_python({
            'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'MANUAL',
        })
        command = TypeAdapter(ClientMessage).validate_python({
            'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'FORWARD',
        })
        self.assertEqual(mode.type, 'ROBOT_MODE')
        self.assertEqual(command.type, 'ROBOT_MANUAL')
        traced = TypeAdapter(ClientMessage).validate_python({
            'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP',
            'sequence_id': 103, 'client_monotonic': 12.5,
        })
        self.assertEqual(traced.sequence_id, 103)
        self.assertEqual(traced.client_monotonic, 12.5)
        with self.assertRaises(ValueError):
            TypeAdapter(ClientMessage).validate_python({
                'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'STOP',
                'client_monotonic': float('nan'),
            })

    async def test_detail_navigation_messages_are_schema_validated(self):
        goal = TypeAdapter(ClientMessage).validate_python({
            'type': 'NAV_GOAL', 'robot_id': 'R02', 'x': 4.5, 'y': 1.25,
            'yaw': 1.57, 'frame_id': 'map',
        })
        pause = TypeAdapter(ClientMessage).validate_python({
            'type': 'NAV_PAUSE', 'robot_id': 'R02',
        })
        self.assertEqual(goal.type, 'NAV_GOAL')
        self.assertEqual(goal.robot_id, 'R02')
        self.assertEqual(pause.type, 'NAV_PAUSE')

    async def test_gateway_maps_manual_control_actions(self):
        previous = registry.consumer

        class Capture:
            async def send_json(self, payload):
                self.payload = payload

        capture = Capture()
        registry.consumer = capture
        try:
            mode = await RosBridgeGateway().send_command('R01', 'CONTROL_MODE', {'mode': 'MANUAL'})
            manual = await RosBridgeGateway().send_command('R01', 'MANUAL_CMD', {'action': 'STOP'})
            self.assertTrue(mode['ok'])
            self.assertTrue(manual['ok'])
            self.assertEqual(manual['message'], {
                'mode': 'MANUAL', 'action': 'STOP', 'robot_id': 'R01', 'type': 'MANUAL_CMD',
            })
        finally:
            registry.consumer = previous

    async def test_gateway_maps_detail_navigation_actions(self):
        previous = registry.consumer

        class Capture:
            robot_id = 'R02'

            async def send_json(self, payload):
                self.payload = payload

        capture = Capture()
        registry.consumer = capture
        try:
            for action in ('NAV_CANCEL', 'NAV_PAUSE', 'NAV_RESUME'):
                result = await RosBridgeGateway().send_command('R02', action, {})
                self.assertTrue(result['ok'])
                self.assertEqual(result['message'], {'robot_id': 'R02', 'type': action})
        finally:
            registry.consumer = previous


class RosTelemetryTests(IsolatedAsyncioTestCase):
    async def test_external_robot_state_merges_pose_twist_and_metadata(self):
        old_mode = runtime.runtime_mode
        old_operation_mode = runtime.operation_mode
        old_robots = deepcopy(runtime.engine.state['robots'])
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_bridge_connected = runtime.ros_bridge_connected
        old_bridge_status = runtime.bridge_status
        old_ros_diagnostics = deepcopy(runtime.ros_diagnostics)
        old_last_telemetry_at = runtime.last_telemetry_at
        old_last_telemetry_iso = runtime.last_telemetry_iso
        old_last_ros_heartbeat = runtime.last_ros_heartbeat
        old_published_revision = runtime.published_map_revision
        old_map_sync_error = runtime.map_sync_error
        old_robot_map_sync = deepcopy(runtime.robot_map_sync)
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.operation_mode = 'NAVIGATION'
            runtime.connected_robot_ids.add('R01')
            runtime.robot_bridge_heartbeats['R01'] = time.monotonic()
            runtime.ros_bridge_connected = True
            runtime.client_count = 0
            runtime.published_map_revision = 12
            runtime.map_sync_error = None
            runtime.robot_map_sync['R01'] = {
                'ros_revision': 12, 'gazebo_revision': 12,
                'nav2_revision': 12, 'tag_map_revision': 12,
                'tf_status': True, 'status': 'SYNCED',
            }
            telemetry = {
                'type': 'ROBOT_STATE', 'robot_id': 'R01', 'frame_id': 'map',
                'map_revision': 12, 'x': 1.2, 'y': 2.3, 'z': 0.1, 'yaw': 0.4,
                'active_map_id': 'CANONICAL', 'active_map_revision': '12',
                'map_source': 'CANONICAL', 'pose_source': 'TF',
                'vx': 0.5, 'vy': 0.2, 'wz': -0.1,
                'navigation_state': 'NAVIGATING', 'control_mode': 'MANUAL',
                'timestamp': '2026-09-22T00:00:00+00:00',
            }
            await runtime.update_external_robot_state(telemetry)
            runtime.engine.state['robots']['R01'].update({
                'battery': 70, 'health': 'OK', 'load': {'units': 1},
                'stats': {'tasks_done': 10}, 'perception': {'obstacles': 2},
            })
            await runtime.update_external_robot_state(telemetry)
            robot = runtime.engine.state['robots']['R01']
            self.assertEqual(robot['position'], [1.2, 0.1, 2.3])
            self.assertAlmostEqual(robot['heading'], 0.4)
            self.assertEqual(robot['vx'], 0.5)
            self.assertEqual(robot['vy'], 0.2)
            self.assertEqual(robot['wz'], -0.1)
            self.assertEqual(robot['navigation_state'], 'NAVIGATING')
            self.assertEqual(robot['control_mode'], 'MANUAL')
            self.assertEqual(robot['status'], 'ACTIVE')
            self.assertEqual(robot['fsm'], 'NAVIGATING')
            self.assertIsNone(robot['battery'])
            self.assertIsNone(robot['health'])
            self.assertIsNone(robot['load'])
            self.assertIsNone(robot['stats'])
            self.assertIsNone(robot['perception'])
            self.assertEqual(robot['pose_frame_id'], 'map')
            self.assertEqual(robot['pose_map_id'], 'CANONICAL')
            self.assertEqual(robot['pose_map_revision'], '12')
            self.assertEqual(robot['pose_map_source'], 'CANONICAL')
            self.assertEqual(robot['pose_source'], 'TF')
            self.assertIsNone(robot['pose_mapping_session_id'])
            previous_robot_patch = runtime._prev.setdefault('_robots', {}).pop('R01', None)
            robot_patch = runtime._robot_patch()['R01']
            self.assertEqual(robot_patch['pose_frame_id'], 'map')
            self.assertEqual(robot_patch['pose_map_id'], 'CANONICAL')
            self.assertEqual(robot_patch['pose_map_revision'], '12')
            if previous_robot_patch is not None:
                runtime._prev['_robots']['R01'] = previous_robot_patch
            else:
                runtime._prev['_robots'].pop('R01', None)
            self.assertTrue(runtime.ros_bridge_connected)
            self.assertEqual(runtime.last_telemetry_iso, '2026-09-22T00:00:00+00:00')
        finally:
            runtime.runtime_mode = old_mode
            runtime.operation_mode = old_operation_mode
            runtime.engine.state['robots'] = old_robots
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats
            runtime.ros_bridge_connected = old_bridge_connected
            runtime.bridge_status = old_bridge_status
            runtime.ros_diagnostics = old_ros_diagnostics
            runtime.last_telemetry_at = old_last_telemetry_at
            runtime.last_telemetry_iso = old_last_telemetry_iso
            runtime.last_ros_heartbeat = old_last_ros_heartbeat
            runtime.published_map_revision = old_published_revision
            runtime.map_sync_error = old_map_sync_error
            runtime.robot_map_sync = old_robot_map_sync

    async def test_external_pose_rejects_odometry_frame_and_wrong_revision(self):
        old_mode = runtime.runtime_mode
        old_operation_mode = runtime.operation_mode
        old_published = runtime.published_map_revision
        old_error = runtime.map_sync_error
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_bridge_connected = runtime.ros_bridge_connected
        old_map_tf_status = runtime.map_tf_status
        robots = runtime.engine.state.setdefault('robots', {})
        old_robot = deepcopy(robots.get('R01'))
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.operation_mode = 'NAVIGATION'
            runtime.connected_robot_ids.add('R01')
            runtime.robot_bridge_heartbeats['R01'] = time.monotonic()
            runtime.ros_bridge_connected = True
            runtime.published_map_revision = 12
            runtime.map_sync_error = None
            await runtime.update_external_robot_state({
                'robot_id': 'R01', 'frame_id': 'odom', 'map_revision': 12,
                'x': 99, 'y': 99, 'yaw': 0, 'vx': 0, 'vy': 0, 'wz': 0,
            })
            if old_robot is None:
                self.assertNotIn('R01', runtime.engine.state['robots'])
            else:
                self.assertNotEqual(runtime.engine.state['robots']['R01']['position'][0], 99)
            self.assertIn('frame must be map', runtime.map_sync_error)
            runtime.map_sync_error = None
            await runtime.update_external_robot_state({
                'robot_id': 'R01', 'frame_id': 'map', 'map_revision': 11,
                'x': 99, 'y': 99, 'yaw': 0, 'vx': 0, 'vy': 0, 'wz': 0,
            })
            if old_robot is None:
                self.assertNotIn('R01', runtime.engine.state['robots'])
            else:
                self.assertNotEqual(runtime.engine.state['robots']['R01']['position'][0], 99)
            self.assertIn('does not match published revision', runtime.map_sync_error)
        finally:
            runtime.runtime_mode = old_mode
            runtime.operation_mode = old_operation_mode
            runtime.published_map_revision = old_published
            runtime.map_sync_error = old_error
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats
            runtime.ros_bridge_connected = old_bridge_connected
            runtime.map_tf_status = old_map_tf_status
            if old_robot is None:
                runtime.engine.state['robots'].pop('R01', None)
            else:
                runtime.engine.state['robots']['R01'] = old_robot

    async def test_mapping_pose_requires_current_authenticated_slam_session(self):
        old_mode = runtime.runtime_mode
        old_operation_mode = runtime.operation_mode
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_sessions = runtime.robot_mapping_sessions.copy()
        old_robot_modes = runtime.robot_runtime_modes.copy()
        old_published = runtime.published_map_revision
        robots = runtime.engine.state.setdefault('robots', {})
        old_robot = deepcopy(robots.get('R01'))
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.operation_mode = 'MAPPING'
            runtime.published_map_revision = 21
            runtime.connected_robot_ids.add('R01')
            runtime.robot_mapping_sessions['R01'] = 'session-current'
            runtime.robot_runtime_modes['R01'] = 'MAPPING'
            await runtime.update_external_robot_state({
                'robot_id': 'R01', 'frame_id': 'map', 'map_revision': 21,
                'active_map_id': 'SLAM-session-current', 'active_map_revision': 'grid-abc',
                'mapping_session_id': 'session-current', 'map_source': 'SLAM_TOOLBOX',
                'pose_source': 'TF',
                'canonical_pose': {'x': 15, 'y': 5.5, 'yaw': 0.8, 'frame_id': 'map',
                    'map_id': 'CANONICAL', 'map_revision': '21', 'map_source': 'CANONICAL',
                    'pose_source': 'GAZEBO_MODEL_STATES', 'source_frame_id': 'world',
                    'transform_source': 'VALIDATED_CANONICAL_WORLD_BUNDLE', 'valid': True,
                    'timestamp': '2026-10-02T00:00:00+00:00'},
                'x': 1.2, 'y': 2.3, 'z': 0.0, 'yaw': 0.4, 'vx': 0, 'vy': 0, 'wz': 0,
            })
            self.assertEqual(runtime.engine.state['robots']['R01']['position'], [1.2, 0.0, 2.3])
            self.assertEqual(runtime.engine.state['robots']['R01']['pose_map_id'], 'SLAM-session-current')
            self.assertEqual(runtime.engine.state['robots']['R01']['pose_map_revision'], 'grid-abc')
            self.assertEqual(runtime.engine.state['robots']['R01']['pose_map_source'], 'SLAM_TOOLBOX')
            self.assertEqual(runtime.engine.state['robots']['R01']['pose_mapping_session_id'], 'session-current')
            self.assertEqual(runtime.engine.state['robots']['R01']['active_map_pose']['map_id'], 'SLAM-session-current')
            self.assertEqual(runtime.engine.state['robots']['R01']['active_map_pose']['x'], 1.2)
            self.assertEqual(runtime.engine.state['robots']['R01']['slam_pose']['x'], 1.2)
            self.assertEqual(runtime.engine.state['robots']['R01']['canonical_pose']['x'], 15)
            self.assertEqual(runtime.engine.state['robots']['R01']['canonical_pose']['yaw'], 0.8)
            await runtime.update_external_robot_state({
                'robot_id': 'R01', 'frame_id': 'map', 'map_revision': 21,
                'active_map_id': 'SLAM-session-current', 'active_map_revision': 'grid-abc',
                'mapping_session_id': 'session-stale',
                'x': 99, 'y': 99, 'yaw': 0, 'vx': 0, 'vy': 0, 'wz': 0,
            })
            self.assertEqual(runtime.engine.state['robots']['R01']['position'], [1.2, 0.0, 2.3])
        finally:
            runtime.runtime_mode = old_mode
            runtime.operation_mode = old_operation_mode
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats
            runtime.robot_mapping_sessions.clear(); runtime.robot_mapping_sessions.update(old_sessions)
            runtime.robot_runtime_modes.clear(); runtime.robot_runtime_modes.update(old_robot_modes)
            runtime.published_map_revision = old_published
            if old_robot is None:
                robots.pop('R01', None)
            else:
                robots['R01'] = old_robot

    async def test_unconnected_heartbeat_cannot_create_a_false_robot_connection(self):
        old_mode = runtime.runtime_mode
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_bridge_connected = runtime.ros_bridge_connected
        old_r02_bridge = registry.consumers.pop('R02', None)
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.connected_robot_ids = {'R01'}
            runtime.robot_bridge_heartbeats = {'R01': time.monotonic()}
            runtime.ros_bridge_connected = True
            await runtime.handle_ros_message({'type': 'HEARTBEAT', 'robot_id': 'R02'})
            self.assertEqual(runtime.online_robot_ids(), ['R01'])
            self.assertNotIn('R02', runtime.connected_robot_ids)
        finally:
            runtime.runtime_mode = old_mode
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats
            runtime.ros_bridge_connected = old_bridge_connected
            if old_r02_bridge is not None:
                registry.consumers['R02'] = old_r02_bridge

    async def test_live_authenticated_bridge_heartbeat_restores_its_robot_id(self):
        old_mode = runtime.runtime_mode
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_bridge_connected = runtime.ros_bridge_connected
        old_bridge_status = runtime.bridge_status
        old_nav2_state = runtime.nav2_state
        old_diagnostics = deepcopy(runtime.ros_diagnostics)
        old_r01_bridge = registry.consumers.get('R01')

        class LiveBridge:
            robot_id = 'R01'

        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.connected_robot_ids.clear()
            runtime.robot_bridge_heartbeats.clear()
            runtime.ros_bridge_connected = False
            runtime.bridge_status = 'DISCONNECTED'
            registry.consumers['R01'] = LiveBridge()

            await runtime.handle_ros_message({
                'type': 'HEARTBEAT', 'robot_id': 'R01',
                'bridge_state': 'CONNECTED', 'nav2_state': 'ACTIVE',
            })

            self.assertEqual(runtime.online_robot_ids(), ['R01'])
            self.assertTrue(runtime.ros_bridge_connected)
            self.assertEqual(runtime.bridge_status, 'CONNECTED')
            self.assertNotIn('R02', runtime.connected_robot_ids)
        finally:
            runtime.runtime_mode = old_mode
            runtime.connected_robot_ids = old_connected
            runtime.robot_bridge_heartbeats = old_heartbeats
            runtime.ros_bridge_connected = old_bridge_connected
            runtime.bridge_status = old_bridge_status
            runtime.nav2_state = old_nav2_state
            runtime.ros_diagnostics = old_diagnostics
            if old_r01_bridge is None:
                registry.consumers.pop('R01', None)
            else:
                registry.consumers['R01'] = old_r01_bridge


class RosBridgeIdentityTests(IsolatedAsyncioTestCase):
    async def test_bridge_cannot_report_another_robot_id(self):
        bridge = object.__new__(RosBridgeConsumer)
        bridge.robot_id = 'R01'
        bridge._message_window_started = time.monotonic()
        bridge._message_window_count = 0
        bridge.send_json = AsyncMock()
        with patch('twin.ros_bridge_consumer.runtime.handle_ros_message', new=AsyncMock()) as handle:
            await RosBridgeConsumer.receive_json(bridge, {
                'type': 'HEARTBEAT', 'robot_id': 'R02',
            })
        handle.assert_not_awaited()
        bridge.send_json.assert_awaited_once()
        self.assertEqual(bridge.send_json.await_args.args[0]['code'], 'ROBOT_ID_MISMATCH')
