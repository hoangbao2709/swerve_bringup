import copy
import json
from pathlib import Path

from django.contrib.auth.models import User
from django.test import TestCase

from twin.models import Warehouse, Zone, Shelf, WarehouseMap
from twin.warehouse_services import (
    sync_from_layout, ensure_active_map, commit_master_to_map,
    save_layout_to_map, publish_layout_to_map,
)


class WarehouseMapSyncTests(TestCase):
    def setUp(self):
        self.user = User.objects.create_user(username="admin2", password="admin12345")
        layout_path = Path(__file__).resolve().parents[1] / "warehouse_layout.json"
        self.layout = json.loads(layout_path.read_text(encoding="utf-8"))
        result = sync_from_layout(self.layout)
        self.warehouse = Warehouse.objects.get(pk=result["warehouse_id"])
        self.map = ensure_active_map(self.layout)

    def test_crud_master_change_updates_map_geometry(self):
        shelf = Shelf.objects.filter(zone__warehouse=self.warehouse).first()
        self.assertIsNotNone(shelf)
        shelf.position_x = 12.345
        shelf.position_y = 22.678
        shelf.access_x = 11.9
        shelf.access_y = 22.1
        shelf.save()
        before = self.map.revision
        updated = commit_master_to_map(self.warehouse, user=self.user, fallback_layout=self.layout)
        self.assertGreater(updated.revision, before)
        rack = next(r for r in updated.layout["racks"] if r["id"] == (shelf.layout_rack_id or shelf.code))
        self.assertEqual(rack["position"][0], 12.345)
        self.assertEqual(rack["position"][2], 22.678)

    def test_editor_save_updates_master_data(self):
        doc = copy.deepcopy(self.map.layout)
        rack = doc["racks"][0]
        rack["position"][0] = 7.125
        rack["position"][2] = 9.875
        updated = save_layout_to_map(doc, user=self.user, fallback_layout=self.layout)
        shelf = Shelf.objects.get(layout_rack_id=rack["id"])
        self.assertAlmostEqual(shelf.position_x, 7.125)
        self.assertAlmostEqual(shelf.position_y, 9.875)
        self.assertEqual(updated.layout["racks"][0]["position"][0], 7.125)

    def test_editor_publish_creates_version(self):
        doc = copy.deepcopy(self.map.layout)
        published = publish_layout_to_map(doc, user=self.user, fallback_layout=self.layout)
        self.assertEqual(published.published_version, 1)
        self.assertEqual(published.versions.count(), 1)

    def test_each_warehouse_has_independent_map(self):
        other = Warehouse.objects.create(
            code="WH-SECOND", name="Second", width=20, depth=10, height=6, units="m"
        )
        other_map = commit_master_to_map(other, user=self.user, fallback_layout=None)
        self.assertNotEqual(other_map.warehouse_id, self.map.warehouse_id)
        self.assertEqual(other_map.layout["size"]["width"], 20.0)
        self.assertEqual(other_map.layout["racks"], [])
        self.map.refresh_from_db()
        self.assertTrue(self.map.is_active)
        self.assertFalse(other_map.is_active)
