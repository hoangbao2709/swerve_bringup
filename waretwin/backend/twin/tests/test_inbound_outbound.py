from django.test import TestCase

from twin.models import Shelf, Warehouse, WarehouseOrder, WorkPoint, Zone
from twin.schedule_services import apply_completed_flow_to_shelf, create_order


class InboundOutboundFlowTests(TestCase):
    def setUp(self):
        self.wh = Warehouse.objects.create(code="WH-FLOW", name="Flow Test", width=100, depth=70, height=12)
        self.zone = Zone.objects.create(warehouse=self.wh, code="A", name="A")
        self.shelf = Shelf.objects.create(
            zone=self.zone, code="RACK-A101", name="Rack A101", layout_rack_id="RACK-A101",
            capacity=8, current_load=2, status="OCCUPIED",
        )
        self.inbound = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code="INBOUND-1", name="Inbound 1", kind="INBOUND",
            resource_type="INBOUND", resource_id="INBOUND-1",
        )
        self.outbound = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code="OUTBOUND-1", name="Outbound 1", kind="OUTBOUND",
            resource_type="OUTBOUND", resource_id="OUTBOUND-1",
        )
        self.shelf_point = WorkPoint.objects.create(
            warehouse=self.wh, zone=self.zone, code="SHELF-A01", name="Shelf A01", kind="SHELF",
            resource_type="SHELF", resource_id="RACK-A101",
        )

    def test_inbound_and_outbound_routes_are_validated(self):
        inbound = create_order({
            "warehouse_id": self.wh.id, "type": "INBOUND",
            "source_id": self.inbound.id, "destination_id": self.shelf_point.id,
        })
        self.assertEqual(inbound.type, "INBOUND")
        outbound = create_order({
            "warehouse_id": self.wh.id, "type": "OUTBOUND",
            "source_id": self.shelf_point.id, "destination_id": self.outbound.id,
        })
        self.assertEqual(outbound.type, "OUTBOUND")
        with self.assertRaises(ValueError):
            create_order({
                "warehouse_id": self.wh.id, "type": "INBOUND",
                "source_id": self.shelf_point.id, "destination_id": self.inbound.id,
            })

    def test_completed_flow_updates_shelf_exactly_once(self):
        order = WarehouseOrder.objects.create(
            warehouse=self.wh, order_no="IN-1", type="INBOUND", status="COMPLETED",
            source=self.inbound, destination=self.shelf_point,
        )
        self.assertTrue(apply_completed_flow_to_shelf(order))
        self.shelf.refresh_from_db()
        self.assertEqual(self.shelf.current_load, 3)
        self.assertFalse(apply_completed_flow_to_shelf(order))
        self.shelf.refresh_from_db()
        self.assertEqual(self.shelf.current_load, 3)
