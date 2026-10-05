from django.test import TestCase
from django.urls import Resolver404, resolve


class RetainedProductApiSurfaceTests(TestCase):
    removed_paths = (
        '/api/auth/login',
        '/api/auth/logout',
        '/api/auth/me',
        '/api/auth/register',
        '/api/admin/users',
        '/api/admin/users/1',
        '/api/admin/users/1/reset-password',
        '/api/scheduler/sync',
        '/api/warehouses',
        '/api/warehouses/1',
        '/api/warehouse/1/export/gazebo/',
        '/api/zones',
        '/api/zones/1',
        '/api/shelves/1',
        '/api/warehouse-tree',
        '/api/warehouse-sync/from-layout',
        '/api/warehouse-maps',
        '/api/warehouse-maps/1/activate',
        '/api/layout/validate',
        '/api/layout/draft',
        '/api/layout/publish',
        '/api/layout/versions',
        '/api/state',
        '/api/state/validate',
        '/api/kpi',
        '/api/decisions',
        '/api/inject',
        '/api/inject/clear',
        '/api/tasks',
        '/api/tasks/LEGACY-1/assign',
        '/api/copilot',
        '/api/vlm/observe',
        '/api/whatif',
        '/api/ai/status',
        '/api/sim',
    )

    def test_removed_product_endpoints_resolve_to_404_for_read_and_write_methods(self):
        for path in self.removed_paths:
            with self.subTest(path=path):
                for method in (self.client.get, self.client.post):
                    response = method(path, data={})
                    self.assertEqual(response.status_code, 404, f'{path}: {response.status_code}')

    def test_django_admin_is_not_exposed(self):
        for method in (self.client.get, self.client.post):
            self.assertEqual(method('/django-admin/').status_code, 404)

    def test_shelf_catalog_is_read_only(self):
        self.assertEqual(self.client.get('/api/shelves').status_code, 200)
        self.assertEqual(self.client.post('/api/shelves', data={}).status_code, 404)

    def test_overview_and_robot_control_runtime_endpoints_remain_routed(self):
        retained_paths = (
            '/api/health',
            '/api/system/status',
            '/api/layout',
            '/api/map/sync-status',
            '/api/scheduler/overview',
            '/api/conveyors',
            '/api/shelves',
            '/api/robots/R01/navigation-tags',
            '/api/robots/R01/map-registration',
            '/api/robots/R01/emergency-stop',
            '/api/robots/R01/local/maps',
        )
        for path in retained_paths:
            with self.subTest(path=path):
                try:
                    resolve(path)
                except Resolver404 as exc:
                    self.fail(f'retained endpoint {path} no longer resolves: {exc}')

    def test_direct_runtime_apis_work_without_credentials(self):
        for path in ('/api/health', '/api/system/status', '/api/layout'):
            with self.subTest(path=path):
                response = self.client.get(path)
                self.assertNotEqual(response.status_code, 401)
