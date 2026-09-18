from django.test import SimpleTestCase

from twin.canonical_map import (
    canonicalize_layout,
    validate_canonical_layout,
    warehouse_to_three,
    warehouse_to_ros,
)


class CanonicalMapTests(SimpleTestCase):
    def test_legacy_size_becomes_rectangular_floor(self):
        result = canonicalize_layout({"size": {"width": 20, "depth": 10}, "floors": [{"id": 1}]})
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(result["coordinate_system"]["frame"], "warehouse_map")
        self.assertEqual(result["floors"][0]["boundary"], [[0, 0], [20, 0], [20, 10], [0, 10]])

    def test_concave_floor_and_hole_are_valid(self):
        layout = {
            "size": {"width": 30, "depth": 30},
            "floors": [{
                "id": "floor-1",
                "boundary": [[0, 0], [20, 0], [20, 10], [30, 10], [30, 30], [0, 30]],
                "holes": [[[5, 5], [8, 5], [8, 8], [5, 8]]],
            }],
        }
        self.assertEqual(validate_canonical_layout(layout), [])

    def test_self_intersection_and_outside_hole_are_rejected(self):
        layout = {
            "size": {"width": 10, "depth": 10},
            "floors": [{
                "id": 1,
                "boundary": [[0, 0], [10, 10], [0, 10], [10, 0]],
                "holes": [[[20, 20], [21, 20], [21, 21]]],
            }],
        }
        errors = validate_canonical_layout(layout)
        self.assertTrue(any("self-intersects" in error for error in errors))
        self.assertTrue(any("outside floor" in error for error in errors))

    def test_floor_references_and_tag_ids_are_validated(self):
        layout = {
            "size": {"width": 10, "depth": 10},
            "floors": [{"id": 1}],
            "navigation_tags": [
                {"uuid": "a", "tag_id": 101, "floor_id": 1},
                {"uuid": "b", "tag_id": 101, "floor_id": 99},
            ],
        }
        errors = validate_canonical_layout(layout)
        self.assertTrue(any("invalid floor reference" in error for error in errors))
        self.assertTrue(any("duplicate tag_id" in error for error in errors))

    def test_coordinate_convention_is_explicit(self):
        self.assertEqual(warehouse_to_three(1, 2, 3), (1.0, 3.0, 2.0))
        self.assertEqual(warehouse_to_ros(1, 2, 3), (1.0, 2.0, 3.0))
