from copy import deepcopy
from unittest import IsolatedAsyncioTestCase
from pydantic import TypeAdapter

from twin.coordinates import ros_pose_to_waretwin, ros_twist_to_waretwin
from twin.gateways.ros_bridge import RosBridgeGateway
from twin.ros_bridge_consumer import registry
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
            self.assertEqual(manual['message'], {'mode': 'MANUAL', 'action': 'STOP', 'robot_id': 'R01', 'type': 'MANUAL_CMD'})
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
        old_bridge_connected = runtime.ros_bridge_connected
        old_bridge_status = runtime.bridge_status
        old_ros_diagnostics = deepcopy(runtime.ros_diagnostics)
        old_last_telemetry_at = runtime.last_telemetry_at
        old_last_telemetry_iso = runtime.last_telemetry_iso
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.client_count = 0
            await runtime.update_external_robot_state({
                'type': 'ROBOT_STATE',
                'robot_id': 'R01',
                'x': 1.2,
                'y': 2.3,
                'z': 0.1,
                'yaw': 0.4,
                'vx': 0.5,
                'vy': 0.2,
                'wz': -0.1,
                'navigation_state': 'NAVIGATING',
                'control_mode': 'MANUAL',
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
            runtime.ros_bridge_connected = old_bridge_connected
            runtime.bridge_status = old_bridge_status
            runtime.ros_diagnostics = old_ros_diagnostics
            runtime.last_telemetry_at = old_last_telemetry_at
            runtime.last_telemetry_iso = old_last_telemetry_iso
