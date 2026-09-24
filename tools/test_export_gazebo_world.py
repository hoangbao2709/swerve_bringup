#!/usr/bin/env python3
import json
import tempfile
import unittest
from pathlib import Path

try:
    from tools.export_gazebo_world import ExportValidationError, export_gazebo_world
except ModuleNotFoundError:  # direct ``python tools/test_export_gazebo_world.py``
    from export_gazebo_world import ExportValidationError, export_gazebo_world


def fixture_layout():
    return {
        "schema_version": 2,
        "id": "export-fixture",
        "name": "Exporter fixture",
        "units": "m",
        "size": {"width": 12, "depth": 10, "height": 8},
        "floors": [
            {
                "id": "F1",
                "name": "Floor 1",
                "elevation": 0,
                "boundary": [[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]],
                "holes": [[[1, 1], [2, 1], [2, 2], [1, 2]]],
            },
            {"id": "F2", "name": "Floor 2", "elevation": 4, "boundary": [[0, 0], [8, 0], [8, 8], [0, 8]], "holes": []},
        ],
        "racks": [
            {"id": "R1", "zone": "", "position": [4, 0, 0], "size": [1, 1, 1], "rotation": 0, "levels": 1, "model": "rack", "blocks_grid": True, "floor": "F1"},
            {"id": "R2", "zone": "", "position": [2, 0, 2], "size": [2, 1, 1], "rotation": 90, "levels": 1, "model": "rack", "blocks_grid": True, "floor": "F2"},
        ],
        "stations": [], "conveyors": [], "obstacles": [{"id": "O1", "kind": "OBSTACLE", "rect": [5, 1, 6, 2], "floor": "F1"}], "columns": [],
        "navigation_tags": [
            {"uuid": "tag-f1-a", "tag_id": 1001, "floor_id": "F1", "x": 5, "y": 1, "z": 0, "yaw": 0},
            {"uuid": "tag-f1-b", "tag_id": 1002, "floor_id": "F1", "x": 6, "y": 1, "z": 0, "yaw": 1.57079632679},
            {"uuid": "tag-f2-a", "tag_id": 2001, "floor_id": "F2", "x": 2, "y": 6, "z": 0, "yaw": 3.14159265359},
        ],
        "aisles": [], "navigation_edges": [], "zones": [], "docks": [], "charging_stations": [], "parking": [], "restricted_areas": [], "walkways": [], "cameras": [], "sensors": [], "locations": [], "lifts": [], "spawn": {"robots": []},
    }


class GazeboExporterTest(unittest.TestCase):
    def test_generates_geometry_tags_manifest_and_is_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = export_gazebo_world(fixture_layout(), root / "one")
            world_one = (root / "one" / "warehouse.world").read_bytes()
            mesh_one = (root / "one" / "models" / "floor_F1" / "mesh.obj").read_bytes()
            second = export_gazebo_world(fixture_layout(), root / "two")
            # The world contains output-specific absolute mesh URIs; the
            # generated geometry and manifest are deterministic independently.
            world_two = (root / "two" / "warehouse.world").read_text().replace(str(root / "two"), "OUTPUT").encode()
            world_one_normalized = world_one.decode().replace(str(root / "one"), "OUTPUT").encode()
            self.assertEqual(world_one_normalized, world_two)
            self.assertEqual(mesh_one, (root / "two" / "models" / "floor_F1" / "mesh.obj").read_bytes())
            self.assertEqual(first["map_revision"], second["map_revision"])

            world = (root / "one" / "warehouse.world").read_text()
            manifest = json.loads((root / "one" / "manifest.json").read_text())
            self.assertIn('model name="floor_F1"', world)
            self.assertIn('<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>', world)
            self.assertIn('model name="floor_F2"', world)
            self.assertEqual(world.count("floor_F1_boundary_wall_"), 6)
            self.assertEqual(world.count("floor_F1_hole_0_wall_"), 4)
            self.assertIn('model name="rack_R1"', world)
            self.assertIn('<pose>4.5 0.5 0.5 0 0 0</pose>', world)
            self.assertIn('model name="nav_tag_2001"', world)
            self.assertIn('<pose>2 6 4 0 0 3.14159265</pose>', world)
            self.assertEqual(len(manifest["floors"]), 2)
            self.assertEqual(len(manifest["tags"]), 3)
            self.assertEqual(next(tag for tag in manifest["tags"] if tag["tag_id"] == 2001)["pose"][2], 4.0)
            self.assertTrue((root / "one" / "models" / "floor_F1" / "mesh.obj").exists())

    def test_exports_robot_spawn_pose_from_canonical_map(self):
        layout = fixture_layout()
        # Use a simple usable rectangle for this pose-focused case while
        # retaining the canonical [x, vertical, floor_y] representation.
        layout["floors"][0]["boundary"] = [[0, 0], [20, 0], [20, 8], [0, 8]]
        layout["spawn"] = {"robots": [{
            "id": "R01", "position": [15, 0.2, 5.5],
            "heading": 1.57079632679, "battery": 100, "floor": "F1",
        }]}
        with tempfile.TemporaryDirectory() as directory:
            manifest = export_gazebo_world(layout, Path(directory) / "generated")
        self.assertEqual(manifest["robots"], [{
            "id": "R01", "floor_id": "F1",
            "pose": [15.0, 5.5, 0.2, 1.57079632679], "battery": 100,
        }])

    def test_spawn_pose_changes_manifest(self):
        layout = fixture_layout()
        layout["spawn"] = {"robots": [{"id": "R01", "position": [2, 0.2, 5], "heading": 0, "floor": "F1"}]}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = export_gazebo_world(layout, root / "first")
            layout["spawn"]["robots"][0]["position"][0] = 2.5
            second = export_gazebo_world(layout, root / "second")
        self.assertNotEqual(first["robots"], second["robots"])

    def test_invalid_layout_does_not_create_output(self):
        invalid = fixture_layout()
        invalid["navigation_tags"][0]["floor_id"] = "missing-floor"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated"
            with self.assertRaises(ExportValidationError):
                export_gazebo_world(invalid, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
