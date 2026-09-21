from unittest.mock import patch

from django.test import TestCase, override_settings

from twin.runtime import runtime


class HealthApiTests(TestCase):
    def test_health_has_real_component_contract(self):
        response = self.client.get('/api/health/')
        self.assertEqual(response.status_code, 200)
        payload = response.json()
        for key in (
            'status', 'backend_status', 'backend_ready', 'system_status', 'system_ready',
            'database', 'ros_bridge', 'websocket', 'ros', 'gazebo',
            'mode', 'runtime_state', 'timestamp', 'version', 'components',
        ):
            self.assertIn(key, payload)
        self.assertTrue(payload['database'])
        self.assertEqual(payload['mode'], payload['runtime_state'])
        self.assertIn(payload['status'], ('ok', 'degraded', 'error'))

    def test_system_status_does_not_fabricate_ros_health(self):
        response = self.client.get('/api/system/status/')
        self.assertEqual(response.status_code, 200)
        payload = response.json()
        self.assertIn('system', payload)
        self.assertIn('ros', payload)
        self.assertIn('runtime', payload)
        self.assertIn('nodes', payload['ros'])
        self.assertIn('topics', payload['ros'])

    def _health_for_mode(self, mode):
        with (
            override_settings(WARETWIN_RUNTIME_MODE=mode),
            patch.object(runtime, 'runtime_mode', mode),
            patch.object(runtime, 'ros_bridge_connected', False),
            patch.object(runtime, 'last_ros_heartbeat', None),
            patch.object(runtime, 'last_telemetry_at', None),
        ):
            return self.client.get('/api/health/').json()

    def test_local_sim_is_healthy_without_ros_or_gazebo(self):
        payload = self._health_for_mode('LOCAL_SIM')
        self.assertEqual(payload['backend_status'], 'READY')
        self.assertEqual(payload['system_status'], 'OK')
        self.assertTrue(payload['ok'])

    def test_gazebo_ros_is_disconnected_without_bridge_heartbeat(self):
        payload = self._health_for_mode('GAZEBO_ROS')
        self.assertEqual(payload['backend_status'], 'READY')
        self.assertEqual(payload['system_status'], 'DISCONNECTED')
        self.assertFalse(payload['ok'])
        self.assertEqual(payload['status'], 'degraded')

    def test_real_robot_is_disconnected_without_robot_heartbeat(self):
        payload = self._health_for_mode('REAL_ROBOT')
        self.assertEqual(payload['backend_status'], 'READY')
        self.assertEqual(payload['system_status'], 'DISCONNECTED')
        self.assertFalse(payload['ok'])
