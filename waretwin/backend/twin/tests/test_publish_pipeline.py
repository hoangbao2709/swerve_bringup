import copy
import json
import tempfile
from pathlib import Path
from unittest.mock import patch

import yaml
from django.test import RequestFactory, TestCase, override_settings

from twin.models import WarehouseMapVersion
from twin.warehouse_services import ensure_active_map, publish_layout_to_map, sync_from_layout
from twin.map_artifacts import render_datamatrix_yaml, render_tag_graph_yaml
from twin.nav2_export import render_nav2_map
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
        self.assertIn('frame_id: map', render_datamatrix_yaml(layout))
        self.assertIn('frame_id: map', graph)
        self.assertIn('"1001"', graph)
        self.assertIn('neighbors: [1002]', graph)
        self.assertIn('direction: "bidirectional"', graph)

    def test_published_tag_artifacts_preserve_semantic_service_metadata_and_revisions(self):
        layout = layout_fixture()
        layout['revision'] = 23
        layout['tag_graph_revision'] = 'graph-r23'
        layout['navigation_tags'][0].update({'semantic_role': 'shelf_service',
            'metadata': {'orientation_policy': 'SHELF_WIDTH_PARALLEL', 'rack_id': 'rack-A',
                'service_face': 'LONG_AXIS_POSITIVE_END', 'service_standoff_m': 1.0,
                'service_aisle_id': 'A1',
                'service_pose': {'frame_id': 'map', 'map_revision': '23',
                                 'x': 1.0, 'y': 5.0, 'yaw': -1.57079632679}}})
        matrix = yaml.safe_load(render_datamatrix_yaml(layout))
        graph = yaml.safe_load(render_tag_graph_yaml(layout))

        self.assertEqual((matrix['canonical_revision'], matrix['graph_revision']),
                         (23, 'graph-r23'))
        matrix_tag = next(tag for tag in matrix['tags'] if tag['id'] == 1001)
        graph_tag = graph['tags']['1001']
        for tag in (matrix_tag, graph_tag):
            self.assertEqual(tag['semantic_role'], 'shelf_service')
            self.assertEqual(tag['orientation_policy'], 'SHELF_WIDTH_PARALLEL')
            self.assertEqual(tag['rack_id'], 'rack-A')
            self.assertEqual(tag['service_pose']['map_revision'], '23')
            self.assertEqual(tag['service_pose']['yaw'], -1.57079632679)

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
            self.assertTrue((first_dir / "nav2" / "warehouse.yaml").is_file())
            self.assertTrue((first_dir / "nav2" / "warehouse.pgm").is_file())
            manifest = json.loads((first_dir / 'manifest.json').read_text())
            self.assertEqual(manifest['frame_id'], 'map')
            self.assertEqual(manifest['revision'], first.revision)
            self.assertTrue(manifest['generated_at'].endswith('+00:00'))
            self.assertIn('nav2/warehouse.pgm', manifest['sha256'])
            self.assertIn('gazebo/manifest.json', manifest['artifacts']['gazebo_manifest'])
            self.assertEqual(manifest['gazebo_bounds'], {
                'min_x': 0.0, 'min_y': 0.0, 'max_x': 10.0, 'max_y': 10.0,
            })
            gazebo_manifest = json.loads((first_dir / manifest['artifacts']['gazebo_manifest']).read_text())
            self.assertEqual(gazebo_manifest['floors'][0]['boundary'], [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
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

    def test_nav2_raster_uses_negative_world_origin_and_north_up_rows(self):
        layout = {
            'size': {'width': 4, 'depth': 2},
            'floors': [{'id': 'F1', 'boundary': [[-2, -1], [2, -1], [2, 1], [-2, 1]]}],
            'racks': [], 'stations': [], 'obstacles': [], 'columns': [], 'conveyors': [],
        }
        image, yaml_text, raster = render_nav2_map(layout, layout['floors'][0], resolution=1)
        header, pixels = image.split(b'255\n', 1)
        self.assertIn(b'4 2', header)
        self.assertEqual(raster['origin'], [-2.0, -1.0, 0.0])
        self.assertIn('origin: [-2, -1, 0.0]', yaml_text)
        self.assertEqual(len(pixels), 8)
        # Top row and bottom row are both inside the polygon; borders are not
        # spuriously displaced by an image-space Y inversion.
        self.assertEqual(set(pixels), {254})
