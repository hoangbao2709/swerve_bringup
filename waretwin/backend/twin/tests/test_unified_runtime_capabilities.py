import time
from contextlib import ExitStack
from types import SimpleNamespace
from unittest import IsolatedAsyncioTestCase, TestCase
from unittest.mock import AsyncMock, patch

from twin.runtime import runtime


def registered_navigation_map(now, session='session-1', registration_revision=3):
    return {
        'ready': True,
        'navigation_map_source': 'PUBLISHED_CANONICAL_REGISTERED',
        'navigation_map_id': f'NAV-21-SLAM-{session}',
        'navigation_map_revision': (
            f'session-{session}:canonical-21:registration-{registration_revision}'),
        'canonical_map_revision': 21,
        'active_map_id': f'SLAM-{session}',
        'active_map_revision': f'session-{session}',
        'registration_revision': registration_revision,
        'registration_source': 'GAZEBO_CANONICAL_ALIGNMENT',
        'frame_id': 'map', 'resolution': 0.05, 'width': 1200, 'height': 600,
        'received_monotonic': now,
    }


class UnifiedRuntimeCapabilityTests(TestCase):
    def configured_runtime(self, *, control_mode='AUTONOMOUS', nav2_ready=True, estop_active=False):
        now = time.monotonic()
        map_payload = {'map': {
            'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'session-1',
            'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-session-1',
            'map_content_revision': 'cells-a1',
        }}
        patches = [
            patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'),
            patch.object(runtime, 'operation_mode', 'UNIFIED'),
            patch.object(runtime, 'published_map_revision', 21),
            patch.object(runtime, 'robot_runtime_modes', {'R01': 'UNIFIED'}),
            patch.object(runtime, 'robot_mapping_sessions', {'R01': 'session-1'}),
            patch.object(runtime, 'robot_navigation_maps', {
                'R01': registered_navigation_map(now),
            }),
            patch.object(runtime, 'robot_map_registrations', {'R01': {
                'canonical_map_revision': '21', 'registration_revision': 3,
                'source': 'GAZEBO_CANONICAL_ALIGNMENT',
            }}),
            patch.object(runtime, 'robot_slam_map_snapshots', {'R01': map_payload}),
            patch.object(runtime, 'robot_mapping_state', {'R01': 'MAPPING'}),
            patch.object(runtime, 'ros_diagnostics', {
                'slam': True, 'nav2': True, 'nav2_ready': nav2_ready,
                'mapping': {'slam_state': 'ACTIVE', 'map_live': True},
            }),
            patch.object(runtime, 'ros_diagnostics_received_monotonic', time.monotonic()),
            patch.object(runtime, 'robot_command_diagnostics', {'R01': {
                'active_control_mode': control_mode, 'estop_active': estop_active,
                'received_monotonic': now,
            }}),
            patch.object(runtime, 'command_ownership', {}),
            patch.object(runtime, 'control_mode_requests', {}, create=True),
            patch.object(runtime, 'local_map_overrides', {}),
            patch.object(runtime, 'local_map_transitions', set()),
            patch.object(runtime, 'engine', SimpleNamespace(state={
                'robots': {'R01': {'control_mode': control_mode}},
            })),
            patch.object(runtime, 'robot_bridge_online', return_value=True),
            patch.object(runtime, 'navigation_localization_state', return_value={'frame_id': 'map'}),
        ]
        return patches

    def test_unified_runtime_keeps_mapping_and_nav2_capabilities_independent(self):
        with ExitStack() as stack:
            for item in self.configured_runtime():
                stack.enter_context(item)
            capabilities = runtime.robot_capabilities('R01')

        self.assertTrue(capabilities['mapping_available'])
        self.assertTrue(capabilities['mapping_active'])
        self.assertTrue(capabilities['nav2_available'])
        self.assertTrue(capabilities['nav2_ready'])
        self.assertTrue(capabilities['map_ready'])
        self.assertTrue(capabilities['goal_available'])
        self.assertFalse(capabilities['manual_available'])
        self.assertFalse(capabilities['tag_navigation_available'])

    def test_unified_navigation_gate_requires_ready_nav2_autonomous_and_clear_estop(self):
        cases = (
            ({'nav2_ready': False}, 'NAV2_NOT_READY'),
            ({'control_mode': 'MANUAL'}, 'CONTROL_MODE_NOT_AUTONOMOUS'),
            ({'estop_active': True}, 'ESTOP_STATE_UNCONFIRMED'),
        )
        for overrides, expected_code in cases:
            with self.subTest(expected_code=expected_code), ExitStack() as stack:
                kwargs = {'control_mode': 'AUTONOMOUS', 'nav2_ready': True, 'estop_active': False}
                kwargs.update(overrides)
                for item in self.configured_runtime(**kwargs):
                    stack.enter_context(item)
                reason = runtime.unified_navigation_blocker('R01')
                self.assertTrue(reason.startswith(expected_code + ':'), reason)

    def test_unified_navigation_gate_requires_current_registered_full_map(self):
        with ExitStack() as stack:
            patches = self.configured_runtime()
            for item in patches:
                stack.enter_context(item)
            runtime.robot_navigation_maps['R01']['canonical_map_revision'] = 20
            reason = runtime.unified_navigation_blocker('R01')
        self.assertTrue(reason.startswith('NAVIGATION_MAP_REVISION_MISMATCH:'), reason)

    def test_unified_runtime_status_exposes_capabilities_per_robot(self):
        old_connected = set(runtime.connected_robot_ids)
        try:
            runtime.connected_robot_ids = {'R01'}
            with ExitStack() as stack:
                for item in self.configured_runtime():
                    stack.enter_context(item)
                status = runtime.runtime_status_message()
            self.assertTrue(status['robot_capabilities']['R01']['mapping_active'])
            self.assertTrue(status['robot_capabilities']['R01']['nav2_ready'])
        finally:
            runtime.connected_robot_ids = old_connected


class UnifiedPreviewTests(IsolatedAsyncioTestCase):
    async def test_slam_content_churn_preserves_capability_preview_and_goal_but_new_session_invalidates(self):
        now = time.monotonic()
        map_payload = {'map': {
            'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'session-1',
            'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-session-1',
            'map_content_revision': 'cells-A',
        }}
        robot_pose = {
            'valid': True, 'frame_id': 'map', 'map_id': 'SLAM-session-1',
            'map_revision': 'session-session-1', 'map_source': 'SLAM_TOOLBOX',
            'pose_source': 'TF', 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
        }
        consumer = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        patches = [
            patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'),
            patch.object(runtime, 'operation_mode', 'UNIFIED'),
            patch.object(runtime, 'published_map_revision', 21),
            patch.object(runtime, 'robot_runtime_modes', {'R01': 'UNIFIED'}),
            patch.object(runtime, 'robot_mapping_sessions', {'R01': 'session-1'}),
            patch.object(runtime, 'robot_navigation_maps', {
                'R01': registered_navigation_map(now),
            }),
            patch.object(runtime, 'robot_map_registrations', {'R01': {
                'canonical_map_revision': '21', 'registration_revision': 3,
                'source': 'GAZEBO_CANONICAL_ALIGNMENT',
            }}),
            patch.object(runtime, 'robot_mapping_state', {'R01': 'MAPPING'}),
            patch.object(runtime, 'robot_slam_map_snapshots', {'R01': map_payload}),
            patch.object(runtime, 'ros_diagnostics', {
                'slam': True, 'nav2': True, 'nav2_ready': True,
                'mapping': {'slam_state': 'ACTIVE', 'map_live': True},
            }),
            patch.object(runtime, 'ros_diagnostics_received_monotonic', now),
            patch.object(runtime, 'robot_command_diagnostics', {'R01': {
                'active_control_mode': 'AUTONOMOUS', 'estop_active': False,
                'received_monotonic': now,
            }}),
            patch.object(runtime, 'control_mode_requests', {}, create=True),
            patch.object(runtime, 'local_map_overrides', {}),
            patch.object(runtime, 'local_map_transitions', set()),
            patch.object(runtime, 'engine', SimpleNamespace(state={'robots': {
                'R01': {'control_mode': 'AUTONOMOUS', 'active_map_pose': robot_pose},
            }})),
            patch.object(runtime, 'robot_pose_heartbeats', {'R01': now}),
            patch.object(runtime, 'robot_bridge_online', return_value=True),
            patch.object(runtime, 'gateway', return_value=gateway),
            patch.object(runtime, 'broadcast', new=AsyncMock()),
            patch.object(runtime, 'path_preview_requests', {}),
            patch.object(runtime, 'path_preview_results', {}),
            patch.object(runtime, 'approved_path_previews', {}),
            patch.object(runtime, 'expired_path_previews', {}),
            patch.object(runtime, 'path_preview_invalidations', {}),
        ]
        with ExitStack() as stack:
            for context in patches:
                stack.enter_context(context)

            first_identity = runtime.active_map_state('R01')
            self.assertEqual((first_identity['active_map_id'], first_identity['active_map_revision']),
                             ('SLAM-session-1', 'session-session-1'))
            self.assertEqual(first_identity['map_content_revision'], 'cells-A')
            self.assertTrue(runtime.robot_capabilities('R01')['goal_available'])

            async def request_preview(request_id):
                await runtime.handle_message(consumer, {
                    'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01', 'request_id': request_id,
                    'source_type': 'ACTIVE_MAP_POINT', 'source_map_id': 'SLAM-session-1',
                    'source_map_revision': 'session-session-1',
                    'x': 1.5, 'y': 2.5, 'yaw': 0.25, 'frame_id': 'map',
                    'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-session-1',
                    'map_id': 'SLAM-session-1', 'map_revision': 'session-session-1',
                })

            async def planner_result(request_id, revision):
                await runtime.handle_ros_message({
                    'type': 'PATH_PREVIEW_RESULT', 'robot_id': 'R01', 'request_id': request_id,
                    'status': 'VALID', 'path': [[0.0, 0.0], [1.5, 2.5]],
                    'goal': {'x': 1.5, 'y': 2.5, 'yaw': 0.25},
                    'active_map_id': 'SLAM-session-1',
                    'active_map_revision': 'session-session-1',
                    'map_content_revision': revision,
                    'navigation_map_revision': 'session-session-1:canonical-21:registration-3',
                })

            await request_preview('content-churn-preview')
            self.assertEqual(gateway.send_command.await_args.args[1], 'PATH_PREVIEW')
            map_payload['map']['map_content_revision'] = 'cells-B'
            self.assertEqual(runtime.active_map_state('R01')['active_map_id'], first_identity['active_map_id'])
            self.assertEqual(runtime.active_map_state('R01')['active_map_revision'], first_identity['active_map_revision'])
            self.assertEqual(runtime.active_map_state('R01')['map_content_revision'], 'cells-B')
            self.assertEqual(runtime.navigation_localization_state('R01', runtime.active_map_state('R01'))['map_revision'],
                             first_identity['active_map_revision'])
            self.assertTrue(runtime.robot_capabilities('R01')['goal_available'])
            await planner_result('content-churn-preview', 'cells-B')
            result = runtime.path_preview_results[('R01', 'content-churn-preview')]
            self.assertEqual(result['status'], 'VALID')
            self.assertEqual(result['map_content_revision_at_request'], 'cells-A')
            self.assertEqual(result['map_content_revision'], 'cells-B')

            map_payload['map']['map_content_revision'] = 'cells-C'
            await runtime.handle_message(consumer, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 1.5, 'y': 2.5, 'yaw': 0.25,
                'frame_id': 'map', 'preview_request_id': 'content-churn-preview',
                'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-session-1',
                'source_type': 'ACTIVE_MAP_POINT', 'source_map_id': 'SLAM-session-1',
                'source_map_revision': 'session-session-1',
            })
            self.assertEqual(gateway.send_command.await_args.args[1], 'NAVIGATE')
            self.assertEqual(gateway.send_command.await_args.args[2]['active_map_revision'],
                             first_identity['active_map_revision'])
            self.assertEqual(gateway.send_command.await_args.args[2]['map_content_revision'], 'cells-C')
            self.assertEqual(gateway.send_command.await_args.args[2]['navigation_map_revision'],
                             'session-session-1:canonical-21:registration-3')

            await request_preview('old-session-preview')
            await planner_result('old-session-preview', 'cells-C')
            map_payload['map'].update({
                'mapping_session_id': 'session-2', 'active_map_id': 'SLAM-session-2',
                'active_map_revision': 'session-session-2', 'map_content_revision': 'cells-new-session',
            })
            runtime.robot_mapping_sessions['R01'] = 'session-2'
            runtime.robot_navigation_maps['R01'] = registered_navigation_map(
                time.monotonic(), session='session-2', registration_revision=1)
            runtime.robot_map_registrations['R01'] = {
                'canonical_map_revision': '21', 'registration_revision': 1,
                'source': 'GAZEBO_CANONICAL_ALIGNMENT',
            }
            runtime.engine.state['robots']['R01']['active_map_pose'] = {
                **robot_pose, 'map_id': 'SLAM-session-2', 'map_revision': 'session-session-2',
            }
            runtime.robot_pose_heartbeats['R01'] = time.monotonic()
            changed_identity = runtime.active_map_state('R01')
            self.assertNotEqual(changed_identity['active_map_id'], first_identity['active_map_id'])
            self.assertNotEqual(changed_identity['active_map_revision'], first_identity['active_map_revision'])
            self.assertTrue(runtime.robot_capabilities('R01')['goal_available'])
            consumer.send_json.reset_mock()
            await runtime.handle_message(consumer, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 1.5, 'y': 2.5, 'yaw': 0.25,
                'frame_id': 'map', 'preview_request_id': 'old-session-preview',
                'active_map_id': 'SLAM-session-2', 'active_map_revision': 'session-session-2',
                'source_type': 'ACTIVE_MAP_POINT', 'source_map_id': 'SLAM-session-1',
                'source_map_revision': 'session-session-1',
            })
            self.assertEqual(consumer.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_MAP_MISMATCH')
            self.assertEqual(sum(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list), 1)

    async def test_nav2_preview_is_allowed_while_unified_slam_is_active(self):
        now = time.monotonic()
        active_map = {
            'active_map_id': 'SLAM-session-1', 'active_map_revision': 'session-session-1',
            'map_content_revision': 'cells-a1',
            'canonical_map_revision': 21, 'map_source': 'SLAM_TOOLBOX',
            'map_sync_status': 'LOCAL_ONLY',
        }
        localization = {
            'frame_id': 'map', 'map_id': active_map['active_map_id'],
            'map_revision': active_map['active_map_revision'],
            'map_source': 'SLAM_TOOLBOX', 'pose_source': 'TF',
        }
        consumer = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        patches = [
            patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'),
            patch.object(runtime, 'operation_mode', 'UNIFIED'),
            patch.object(runtime, 'robot_runtime_modes', {'R01': 'UNIFIED'}),
            patch.object(runtime, 'robot_mapping_sessions', {'R01': 'session-1'}),
            patch.object(runtime, 'robot_mapping_state', {'R01': 'MAPPING'}),
            patch.object(runtime, 'robot_slam_map_snapshots', {'R01': {'map': {
                    'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'session-1',
                    'active_map_id': active_map['active_map_id'],
                    'active_map_revision': active_map['active_map_revision'],
                    'map_content_revision': active_map['map_content_revision'],
                }}}),
            patch.object(runtime, 'ros_diagnostics', {
                    'slam': True, 'nav2': True, 'nav2_ready': True,
                    'mapping': {'slam_state': 'ACTIVE', 'map_live': True},
                }),
            patch.object(runtime, 'ros_diagnostics_received_monotonic', now),
            patch.object(runtime, 'robot_command_diagnostics', {'R01': {
                    'active_control_mode': 'AUTONOMOUS', 'estop_active': False,
                    'received_monotonic': now,
                }}),
            patch.object(runtime, 'engine', SimpleNamespace(state={
                    'robots': {'R01': {'control_mode': 'AUTONOMOUS'}},
                })),
            patch.object(runtime, 'control_mode_requests', {}, create=True),
            patch.object(runtime, 'local_map_transitions', set()),
            patch.object(runtime, 'robot_bridge_online', return_value=True),
            patch.object(runtime, 'unified_navigation_blocker', return_value=None),
            patch.object(runtime, 'active_map_state', return_value=active_map),
            patch.object(runtime, 'navigation_localization_state', return_value=localization),
            patch.object(runtime, 'resolve_navigation_target', new=AsyncMock(side_effect=lambda **kw: {
                    'robot_id': kw['robot_id'], 'source_type': 'ACTIVE_MAP_POINT', 'source_id': None,
                    'frame_id': 'map', 'map_id': active_map['active_map_id'],
                    'map_revision': active_map['active_map_revision'],
                    'x': kw['x'], 'y': kw['y'], 'yaw': kw['yaw'], 'metadata': {},
                })),
            patch.object(runtime, 'gateway', return_value=gateway),
            patch.object(runtime, 'path_preview_requests', {}),
            patch.object(runtime, 'path_preview_results', {}),
            patch.object(runtime, 'approved_path_previews', {}),
            patch.object(runtime, 'expired_path_previews', {}),
            patch.object(runtime, 'path_preview_invalidations', {}),
            patch.object(runtime, 'broadcast', new=AsyncMock()),
        ]
        with ExitStack() as stack:
            for context in patches:
                stack.enter_context(context)
            await runtime.handle_message(consumer, {
                'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01', 'request_id': 'unified-preview',
                'x': 1.5, 'y': 2.5, 'yaw': 0.25, 'frame_id': 'map',
                'active_map_id': active_map['active_map_id'],
                'active_map_revision': active_map['active_map_revision'],
            })

            gateway.send_command.assert_awaited_once()
            self.assertEqual(gateway.send_command.await_args.args[1], 'PATH_PREVIEW')
            self.assertEqual(gateway.send_command.await_args.args[2]['active_map_id'], active_map['active_map_id'])
            self.assertFalse(consumer.send_json.await_args_list)
