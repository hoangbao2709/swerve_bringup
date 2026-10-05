import time
from contextlib import ExitStack
from types import SimpleNamespace
from unittest import IsolatedAsyncioTestCase, TestCase
from unittest.mock import AsyncMock, patch

from twin.runtime import runtime


class UnifiedRuntimeCapabilityTests(TestCase):
    def configured_runtime(self, *, control_mode='AUTONOMOUS', nav2_ready=True, estop_active=False):
        now = time.monotonic()
        map_payload = {'map': {
            'map_source': 'SLAM_TOOLBOX', 'mapping_session_id': 'session-1',
            'active_map_id': 'SLAM-session-1', 'active_map_revision': 'cells-a1',
        }}
        patches = [
            patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'),
            patch.object(runtime, 'operation_mode', 'UNIFIED'),
            patch.object(runtime, 'robot_runtime_modes', {'R01': 'UNIFIED'}),
            patch.object(runtime, 'robot_mapping_sessions', {'R01': 'session-1'}),
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
    async def test_nav2_preview_is_allowed_while_unified_slam_is_active(self):
        now = time.monotonic()
        active_map = {
            'active_map_id': 'SLAM-session-1', 'active_map_revision': 'cells-a1',
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
            patch.object(runtime, 'active_map_state', return_value=active_map),
            patch.object(runtime, 'navigation_localization_state', return_value=localization),
            patch.object(runtime, 'resolve_navigation_target', new=AsyncMock(side_effect=lambda **kw: {
                    'robot_id': kw['robot_id'], 'source_type': 'MAP_POINT', 'source_id': None,
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
