import json
from django.contrib.auth.models import User
from django.test import TestCase

from accounts.models import ApiToken, ensure_profile
from twin.models import Warehouse, Zone, Shelf
from twin.warehouse_services import sync_from_layout


class WarehouseCrudTests(TestCase):
    def setUp(self):
        self.user = User.objects.create_user('admin-test', password='test-password-123')
        ensure_profile(self.user, 'admin')
        self.token = ApiToken.issue(self.user).key
        self.auth = {'HTTP_AUTHORIZATION': f'Bearer {self.token}'}

    def post_json(self, path, body):
        return self.client.post(path, data=json.dumps(body), content_type='application/json', **self.auth)

    def test_legacy_warehouse_crud_routes_are_not_public(self):
        for path in ('/api/warehouses', '/api/warehouses/1', '/api/zones', '/api/zones/1',
                     '/api/shelves', '/api/shelves/1'):
            with self.subTest(path=path):
                self.assertEqual(self.client.get(path, **self.auth).status_code, 404)
        self.assertEqual(self.post_json('/api/warehouses', {'code': 'WH-01'}).status_code, 404)
        self.assertEqual(self.post_json('/api/zones', {'code': 'A'}).status_code, 404)
        self.assertEqual(self.post_json('/api/shelves', {'code': 'A-001'}).status_code, 404)

    def test_sync_layout_is_idempotent(self):
        layout = {
            'id':'wh-test','name':'Test','units':'m','size':{'width':20,'depth':20,'height':6},
            'zones':[{'id':'A','name':'Zone A','floor':1,'color':'#3b82f6','polygon':[[0,0],[10,0],[10,10],[0,10]]}],
            'racks':[{'id':'R1','zone':'A','floor':1,'position':[2,0,3],'size':[3,5,1.2],'rotation':0,'levels':4,'model':'rack','blocks_grid':True}],
            'locations':[{'id':'S1','kind':'SHELF','zone':'A','rack_id':'R1','access_point':[2,2]}],
        }
        first = sync_from_layout(layout)
        second = sync_from_layout(layout)
        self.assertEqual({k: first[k] for k in ('warehouses', 'zones', 'shelves')}, {'warehouses':1,'zones':1,'shelves':1})
        self.assertEqual(second, first)
        self.assertEqual(Warehouse.objects.count(), 1)
        self.assertEqual(Zone.objects.count(), 1)
        self.assertEqual(Shelf.objects.count(), 1)
        shelf = Shelf.objects.get(code='R1')
        self.assertEqual(shelf.access_x, 2)
        self.assertEqual(shelf.metadata['access_points'], [[2.0,2.0]])
