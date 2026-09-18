import copy
import json
import tempfile
from pathlib import Path
from unittest.mock import patch

from django.test import RequestFactory, TestCase, override_settings

from twin.models import WarehouseMapVersion
from twin.warehouse_services import ensure_active_map, publish_layout_to_map, sync_from_layout
from twin.map_artifacts import render_datamatrix_yaml, render_tag_graph_yaml
from twin.views import _layout_revision_conflict


def layout_fixture():
    return {
        "schema_version": 2,
        "id": "publish-fixture",
        "name": "Publish fixture",
        "units": "m",
        "size": {"width": 10, "depth": 10, "height": 4},
        "floors": [{"id": "F1", "name": "Floor 1", "elevation": 0}],
        "aisles": [{"id": "A1", "floor_id": "F1", "centerline": [[1, 5], [9, 5]], "width": 1}],
        "navigation_tags": [
            {"uuid": "tag-one", "tag_id": 1001, "floor_id": "F1", "x": 1, "y": 5, "z": 0, "yaw": 0},
            {"uuid": "tag-two", "tag_id": 1002, "floor_id": "F1", "x": 5, "y": 5, "z": 0, "yaw": 0},
            {"uuid": "tag-three", "tag_id": 1003, "floor_id": "F1", "x": 9, "y": 5, "z": 0, "yaw": 0},
        ],
        "navigation_edges": [{
            "uuid": "edge-one", "from_tag_uuid": "tag-one", "to_tag_uuid": "tag-two",
            "from_tag_id": 1001, "to_tag_id": 1002, "aisle_id": "A1", "floor_id": "F1",
            "distance": 4, "cost": 4, "direction": "bidirectional", "enabled": True,
        }],
        "racks": [], "stations": [], "conveyors": [], "obstacles": [], "columns": [],
    }


class PublishPipelineTests(TestCase):
    def test_stale_layout_revision_is_conflict(self):
        request = RequestFactory().put('/api/layout/draft', HTTP_X_LAYOUT_REVISION='4')
        active = type('Map', (), {'revision': 5})()
        response = _layout_revision_conflict(request, active)
        self.assertEqual(response.status_code, 409)

    def test_serializers_are_deterministic_and_compatible(self):
        layout = layout_fixture()
        self.assertEqual(render_datamatrix_yaml(layout), render_datamatrix_yaml(copy.deepcopy(layout)))
        graph = render_tag_graph_yaml(layout)
        self.assertIn('"1001"', graph)
        self.assertIn('neighbors: [1002]', graph)
        self.assertIn('direction: "bidirectional"', graph)

    def test_publish_creates_immutable_revision_artifacts_and_increments(self):
        with tempfile.TemporaryDirectory() as tmp, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tmp)):
            layout = layout_fixture()
            sync_from_layout(layout)
            ensure_active_map(layout)
            first = publish_layout_to_map(layout)
            first_dir = Path(first._artifact_dir)
            first_manifest = (first_dir / "manifest.json").read_text()
            self.assertTrue((first_dir / "canonical_map.json").is_file())
            self.assertTrue((first_dir / "datamatrix_map.yaml").is_file())
            self.assertTrue((first_dir / "tag_graph.yaml").is_file())
            self.assertTrue((first_dir / "gazebo" / "warehouse.world").is_file())
            world_text = (first_dir / "gazebo" / "warehouse.world").read_text()
            self.assertIn(str(first_dir / "gazebo" / "models"), world_text)
            second = publish_layout_to_map(layout)
            self.assertEqual(second.published_version, first.published_version + 1)
            self.assertNotEqual(first.revision, second.revision)
            self.assertEqual((first_dir / "manifest.json").read_text(), first_manifest)
            self.assertEqual(WarehouseMapVersion.objects.count(), 2)

    def test_artifact_failure_rolls_back_database_and_files(self):
        with tempfile.TemporaryDirectory() as tmp, override_settings(WARETWIN_ARTIFACT_ROOT=Path(tmp)):
            layout = layout_fixture()
            sync_from_layout(layout)
            active = ensure_active_map(layout)
            before = (active.revision, active.published_version)
            with patch("twin.map_artifacts.export_gazebo_world", side_effect=RuntimeError("gazebo failed")):
                with self.assertRaises(RuntimeError):
                    publish_layout_to_map(layout)
            active.refresh_from_db()
            self.assertEqual((active.revision, active.published_version), before)
            self.assertEqual(WarehouseMapVersion.objects.count(), 0)
            self.assertEqual(list(Path(tmp).rglob("*")), [])
