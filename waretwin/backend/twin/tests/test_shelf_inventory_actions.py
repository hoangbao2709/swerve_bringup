from django.test import TestCase
from django.utils import timezone

from twin.models import (
    RobotProfile, Shelf, ShelfInventoryItem, Warehouse, WarehouseOrder, WorkPoint, Zone,
)
from twin.order_import_services import auto_schedule_order
from twin.schedule_services import (
    apply_completed_flow_to_shelf, create_order, reserve_inventory_for_order,
)


class DummyEngine:
    def __init__(self, robots):
        self.state = {'robots': robots}


class DummyRuntime:
    def __init__(self, robots):
        self.engine = DummyEngine(robots)


class ShelfInventoryActionTests(TestCase):
    def setUp(self):
        self.wh = Warehouse.objects.create(code='WH-INV', name='Inventory Test', width=100, depth=70, height=12)
        self.zone = Zone.objects.create(warehouse=self.wh, code='A', name='A')
        self.s1 = Shelf.objects.create(
            zone=self.zone, code='RACK-A101', name='Rack A101', layout_rack_id='RACK-A101',
            capacity=8, current_load=2, status='OCCUPIED',
        )
        self.s2 = Shelf.objects.create(
            zone=self.zone, code='RACK-A102', name='Rack A102', layout_rack_id='RACK-A102',
            capacity=8, current_load=0, status='AVAILABLE',
        )
        self.inbound = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code='INBOUND-1', name='Inbound', kind='INBOUND', x=0, y=0,
            resource_type='INBOUND', resource_id='INBOUND-1',
        )
        self.outbound = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code='OUTBOUND-1', name='Outbound', kind='OUTBOUND', x=30, y=0,
            resource_type='OUTBOUND', resource_id='OUTBOUND-1',
        )
        self.p1 = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code='SHELF-A101', name='A101', kind='SHELF', x=10, y=10,
            resource_type='SHELF', resource_id='RACK-A101',
        )
        self.p2 = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code='SHELF-A102', name='A102', kind='SHELF', x=20, y=10,
            resource_type='SHELF', resource_id='RACK-A102',
        )
        self.item = ShelfInventoryItem.objects.create(
            item_uid='ITEM-001', warehouse=self.wh, shelf=self.s1, status='STORED',
            item_code='SKU-001', item_name='Box 001', quantity=1, load_units=1,
        )
        ShelfInventoryItem.objects.create(
            item_uid='ITEM-002', warehouse=self.wh, shelf=self.s1, status='STORED',
            item_code='SKU-002', item_name='Box 002', quantity=1, load_units=1,
        )

    def test_transfer_moves_exact_item_and_both_shelf_counters(self):
        order = create_order({
            'warehouse_id': self.wh.id, 'type': 'TRANSFER',
            'source_id': self.p1.id, 'destination_id': self.p2.id,
            'metadata': {'item_code': 'SKU-001'},
        })
        reserve_inventory_for_order(order, item_id=self.item.id)
        order.status = 'COMPLETED'; order.save(update_fields=['status', 'updated_at'])

        self.assertTrue(apply_completed_flow_to_shelf(order))
        self.s1.refresh_from_db(); self.s2.refresh_from_db(); self.item.refresh_from_db()
        self.assertEqual(self.s1.current_load, 1)
        self.assertEqual(self.s2.current_load, 1)
        self.assertEqual(self.item.shelf_id, self.s2.id)
        self.assertEqual(self.item.status, 'STORED')
        self.assertIsNone(self.item.reserved_by_order_id)
        self.assertFalse(apply_completed_flow_to_shelf(order))

    def test_outbound_removes_exact_item(self):
        order = create_order({
            'warehouse_id': self.wh.id, 'type': 'OUTBOUND',
            'source_id': self.p1.id, 'destination_id': self.outbound.id,
        })
        reserve_inventory_for_order(order, item_id=self.item.id)
        order.status = 'COMPLETED'; order.save(update_fields=['status', 'updated_at'])
        apply_completed_flow_to_shelf(order)

        self.s1.refresh_from_db(); self.item.refresh_from_db()
        self.assertEqual(self.s1.current_load, 1)
        self.assertEqual(self.item.status, 'OUTBOUND')
        self.assertIsNone(self.item.shelf_id)

    def test_inbound_completion_creates_business_item(self):
        self.s2.current_load = 0; self.s2.save(update_fields=['current_load'])
        order = WarehouseOrder.objects.create(
            warehouse=self.wh, order_no='IN-EXACT-1', external_ref='ASN-1', type='INBOUND', status='COMPLETED',
            source=self.inbound, destination=self.p2, quantity=3, load_units=1, payload_weight_kg=2.5,
            metadata={'item_code': 'SKU-NEW', 'item_name': 'New Product'},
        )
        apply_completed_flow_to_shelf(order)
        created = ShelfInventoryItem.objects.get(origin_order=order)
        self.assertEqual(created.shelf_id, self.s2.id)
        self.assertEqual(created.item_code, 'SKU-NEW')
        self.assertEqual(created.external_ref, 'ASN-1')
        self.assertEqual(created.quantity, 3)

    def test_prefer_unloaded_robot_excludes_robot_already_delivering(self):
        order = create_order({
            'warehouse_id': self.wh.id, 'type': 'OUTBOUND',
            'source_id': self.p1.id, 'destination_id': self.outbound.id,
        })
        r1 = RobotProfile.objects.create(warehouse=self.wh, robot_id='R-LOADED', payload_capacity=4)
        r2 = RobotProfile.objects.create(warehouse=self.wh, robot_id='R-PICKING', payload_capacity=4)
        runtime = DummyRuntime({
            r1.robot_id: {
                'status': 'IDLE', 'fsm': 'TRANSPORTING', 'battery': 95, 'floor': 1,
                'position': [9, 0, 10], 'load': {'current': 1, 'capacity': 4}, 'current_task_id': 'T-OLD', 'eta_s': 20,
            },
            r2.robot_id: {
                'status': 'IDLE', 'fsm': 'NAVIGATING', 'battery': 80, 'floor': 1,
                'position': [25, 0, 25], 'load': {'current': 0, 'capacity': 4}, 'current_task_id': 'T-PICK', 'eta_s': 15,
            },
        })
        schedule = auto_schedule_order(
            order, runtime, earliest_start=timezone.now(), prefer_unloaded_robot=True,
        )
        self.assertEqual(schedule.robot.robot_id, 'R-PICKING')
    def test_prefer_unloaded_robot_ranks_idle_before_robot_going_to_pick(self):
        order = create_order({
            'warehouse_id': self.wh.id, 'type': 'OUTBOUND',
            'source_id': self.p1.id, 'destination_id': self.outbound.id,
        })
        idle = RobotProfile.objects.create(warehouse=self.wh, robot_id='R-IDLE', payload_capacity=4)
        picking = RobotProfile.objects.create(warehouse=self.wh, robot_id='R-GOING-PICK', payload_capacity=4)
        runtime = DummyRuntime({
            idle.robot_id: {
                'status': 'IDLE', 'fsm': 'IDLE', 'battery': 80, 'floor': 1,
                'position': [80, 0, 60], 'load': {'current': 0, 'capacity': 4}, 'current_task_id': None,
            },
            picking.robot_id: {
                'status': 'IDLE', 'fsm': 'NAVIGATING', 'battery': 95, 'floor': 1,
                'position': [10, 0, 10], 'load': {'current': 0, 'capacity': 4}, 'current_task_id': 'T-PICK', 'eta_s': 3,
            },
        })
        schedule = auto_schedule_order(
            order, runtime, earliest_start=timezone.now(), prefer_unloaded_robot=True,
        )
        self.assertEqual(schedule.robot.robot_id, 'R-IDLE')

