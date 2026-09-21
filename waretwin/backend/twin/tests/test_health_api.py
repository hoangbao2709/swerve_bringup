from django.test import TestCase


class HealthApiTests(TestCase):
    def test_health_has_real_component_contract(self):
        response = self.client.get('/api/health/')
        self.assertEqual(response.status_code, 200)
        payload = response.json()
        for key in (
            'status', 'database', 'ros_bridge', 'websocket', 'ros', 'gazebo',
            'mode', 'runtime_state', 'timestamp', 'version', 'components',
        ):
            self.assertIn(key, payload)
        self.assertTrue(payload['database'])
        self.assertEqual(payload['mode'], payload['runtime_state'])
        self.assertIn(payload['status'], ('ok', 'degraded'))

    def test_system_status_does_not_fabricate_ros_health(self):
        response = self.client.get('/api/system/status/')
        self.assertEqual(response.status_code, 200)
        payload = response.json()
        self.assertIn('system', payload)
        self.assertIn('ros', payload)
        self.assertIn('runtime', payload)
        self.assertIn('nodes', payload['ros'])
        self.assertIn('topics', payload['ros'])
