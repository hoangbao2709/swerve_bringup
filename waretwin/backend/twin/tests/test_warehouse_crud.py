from django.test import TestCase

from twin.models import Warehouse, Zone, Shelf
from twin.warehouse_services import sync_from_layout


class WarehouseSyncCompatibilityTests(TestCase):
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
