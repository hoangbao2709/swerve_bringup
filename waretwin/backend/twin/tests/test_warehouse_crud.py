import json
from django.contrib.auth.models import User
from django.test import TestCase

from accounts.models import ApiToken, ensure_profile
from twin.models import Warehouse, Zone, Shelf
from twin.warehouse_services import sync_from_layout


class WarehouseCrudTests(TestCase):
    def setUp(self):
        self.user = User.objects.create_user('admin-test', password='admin12345')
        ensure_profile(self.user, 'admin')
        self.token = ApiToken.issue(self.user).key
        self.auth = {'HTTP_AUTHORIZATION': f'Bearer {self.token}'}

    def post_json(self, path, body):
        return self.client.post(path, data=json.dumps(body), content_type='application/json', **self.auth)

    def patch_json(self, path, body):
        return self.client.patch(path, data=json.dumps(body), content_type='application/json', **self.auth)

    def test_full_crud_and_protected_parent_delete(self):
        r = self.post_json('/api/warehouses', {'code':'WH-01','name':'Main','width':100,'depth':70,'height':10})
        self.assertEqual(r.status_code, 201)
        wid = r.json()['id']

        r = self.post_json('/api/zones', {'warehouse_id':wid,'code':'A','name':'Zone A','floor':1,'polygon':[[0,0],[20,0],[20,20],[0,20]]})
        self.assertEqual(r.status_code, 201)
        zid = r.json()['id']

        r = self.post_json('/api/shelves', {
            'zone_id':zid,'code':'A-001','name':'Shelf A-001','capacity':8,'current_load':2,
            'position_x':5,'position_y':5,'position_z':0,'width':3,'depth':1.2,'height':6,
            'access_x':5,'access_y':3.5,'access_yaw':1.57,
        })
        self.assertEqual(r.status_code, 201)
        sid = r.json()['id']

        self.assertEqual(self.client.delete(f'/api/zones/{zid}', **self.auth).status_code, 409)
        self.assertEqual(self.client.delete(f'/api/warehouses/{wid}', **self.auth).status_code, 409)

        r = self.patch_json(f'/api/shelves/{sid}', {'status':'RESERVED','current_load':3})
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['status'], 'RESERVED')
        self.assertEqual(r.json()['current_load'], 3)

        self.assertEqual(self.client.delete(f'/api/shelves/{sid}', **self.auth).status_code, 200)
        self.assertEqual(self.client.delete(f'/api/zones/{zid}', **self.auth).status_code, 200)
        self.assertEqual(self.client.delete(f'/api/warehouses/{wid}', **self.auth).status_code, 200)

    def test_duplicate_codes_are_rejected(self):
        w = Warehouse.objects.create(code='WH', name='Main', width=10, depth=10, height=5)
        Zone.objects.create(warehouse=w, code='A', name='A')
        r = self.post_json('/api/zones', {'warehouse_id':w.id,'code':'A','name':'Duplicate'})
        self.assertEqual(r.status_code, 409)

    def test_shelf_bounds_and_load_validation(self):
        w = Warehouse.objects.create(code='WH', name='Main', width=10, depth=10, height=5)
        z = Zone.objects.create(warehouse=w, code='A', name='A')
        r = self.post_json('/api/shelves', {'zone_id':z.id,'code':'S1','name':'S1','position_x':11,'position_y':1,'access_x':1,'access_y':1})
        self.assertEqual(r.status_code, 400)
        r = self.post_json('/api/shelves', {'zone_id':z.id,'code':'S1','name':'S1','position_x':1,'position_y':1,'access_x':1,'access_y':1,'capacity':2,'current_load':3})
        self.assertEqual(r.status_code, 400)

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
