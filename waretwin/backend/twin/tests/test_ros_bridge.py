from copy import deepcopy
import time
from unittest import IsolatedAsyncioTestCase
from unittest.mock import AsyncMock, patch

from pydantic import TypeAdapter

from twin.coordinates import ros_pose_to_waretwin, ros_twist_to_waretwin
from twin.gateways.ros_bridge import RosBridgeGateway
from twin.ros_bridge_consumer import RosBridgeConsumer, registry
from twin.runtime import runtime
from twin.schema import ClientMessage


class RosCoordinateTests(IsolatedAsyncioTestCase):
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

    async def test_gateway_does_not_publish_without_bridge(self):
        previous = registry.consumer
        registry.consumer = None
        try:
            result = await RosBridgeGateway().send_command('R01', 'NAVIGATE', {'x': 1, 'y': 2, 'yaw': 0})
            self.assertFalse(result['ok'])
            self.assertEqual(result['message']['type'], 'NAV_GOAL')
        finally:
            registry.consumer = previous

    async def test_manual_control_messages_are_schema_validated(self):
        mode = TypeAdapter(ClientMessage).validate_python({
            'type': 'ROBOT_MODE', 'robot_id': 'R01', 'mode': 'MANUAL',
        })
        command = TypeAdapter(ClientMessage).validate_python({
            'type': 'ROBOT_MANUAL', 'robot_id': 'R01', 'action': 'FORWARD',
        })
        self.assertEqual(mode.type, 'ROBOT_MODE')
        self.assertEqual(command.type, 'ROBOT_MANUAL')

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
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.connected_robot_ids.add('R01')
            runtime.robot_bridge_heartbeats['R01'] = time.monotonic()
            runtime.ros_bridge_connected = True
            runtime.client_count = 0
            runtime.published_map_revision = 12
            runtime.map_sync_error = None
            await runtime.update_external_robot_state({
                'type': 'ROBOT_STATE', 'robot_id': 'R01', 'frame_id': 'map',
                'map_revision': 12, 'x': 1.2, 'y': 2.3, 'z': 0.1, 'yaw': 0.4,
                'vx': 0.5, 'vy': 0.2, 'wz': -0.1,
                'navigation_state': 'NAVIGATING', 'control_mode': 'MANUAL',
                'timestamp': '2026-09-22T00:00:00+00:00',
            })
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
            self.assertTrue(runtime.ros_bridge_connected)
            self.assertEqual(runtime.last_telemetry_iso, '2026-09-22T00:00:00+00:00')
        finally:
            runtime.runtime_mode = old_mode
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

    async def test_external_pose_rejects_odometry_frame_and_wrong_revision(self):
        old_mode = runtime.runtime_mode
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

    async def test_unconnected_heartbeat_cannot_create_a_false_robot_connection(self):
        old_mode = runtime.runtime_mode
        old_connected = set(runtime.connected_robot_ids)
        old_heartbeats = dict(runtime.robot_bridge_heartbeats)
        old_bridge_connected = runtime.ros_bridge_connected
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
