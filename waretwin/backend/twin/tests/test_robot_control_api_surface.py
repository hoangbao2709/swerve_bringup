import json

from django.contrib.auth.models import User
from django.test import Client, TestCase


class RobotControlApiSurfaceTests(TestCase):
    def setUp(self):
        User.objects.create_user(username='api-operator', password='test-password-123')
        self.client = Client()
        response = self.client.post('/api/auth/login', data=json.dumps({
            'username': 'api-operator', 'password': 'test-password-123',
        }), content_type='application/json')
        self.assertEqual(response.status_code, 200)
        self.client.defaults['HTTP_AUTHORIZATION'] = f"Bearer {response.json()['access_token']}"

    def test_retired_warehouse_product_endpoints_return_404(self):
        retired_get_routes = (
            '/api/conveyors', '/api/scheduler/overview', '/api/scheduler/workpoints',
            '/api/scheduler/robots', '/api/orders', '/api/schedules',
            '/api/warehouses', '/api/zones', '/api/shelves', '/api/warehouse-tree',
            '/api/warehouse-maps', '/api/admin/users', '/api/state', '/api/kpi',
            '/api/events', '/api/decisions', '/api/tasks', '/api/ai/status', '/api/sim',
            '/api/navigation/missions', '/api/navigation/tag-graph',
        )
        for endpoint in retired_get_routes:
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.client.get(endpoint).status_code, 404)

    def test_retired_motion_and_ai_post_endpoints_return_404(self):
        retired_post_routes = (
            '/api/scheduler/sync', '/api/orders/import', '/api/schedules/create',
            '/api/inject', '/api/inject/clear', '/api/copilot', '/api/vlm/observe', '/api/whatif',
        )
        for endpoint in retired_post_routes:
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.client.post(endpoint, data='{}', content_type='application/json').status_code, 404)
