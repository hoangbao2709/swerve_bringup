from django.test import SimpleTestCase

from twin.canonical_map import (
    aisle_footprint,
    canonicalize_layout,
    validate_aisle_centerline,
    validate_canonical_layout,
    warehouse_to_three,
    warehouse_to_ros,
)


class CanonicalMapTests(SimpleTestCase):
    def test_legacy_size_becomes_rectangular_floor(self):
        result = canonicalize_layout({"size": {"width": 20, "depth": 10}, "floors": [{"id": 1}]})
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(result["coordinate_system"]["frame"], "map")
        self.assertEqual(result["frame_id"], "map")
        self.assertEqual(result["origin"], {"x": 0.0, "y": 0.0})
        self.assertEqual((result["width"], result["height"]), (20.0, 10.0))

    def test_negative_nonzero_floor_bounds_are_preserved_as_world_origin(self):
        result = canonicalize_layout({"size": {"width": 8, "depth": 6}, "floors": [{
            "id": "F1", "boundary": [[-4, -3], [4, -3], [4, 3], [-4, 3]],
        }]})
        self.assertEqual(result["origin"], {"x": -4.0, "y": -3.0})
        self.assertEqual((result["width"], result["height"]), (8.0, 6.0))
        self.assertEqual(result["floors"][0]["boundary"], [[-4, -3], [4, -3], [4, 3], [-4, 3]])

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

    def test_aisle_centerline_and_width_are_validated(self):
        layout = {
            "size": {"width": 20, "depth": 20},
            "floors": [{"id": 1}],
            "aisles": [{"id": "a-1", "floor_id": 1, "centerline": [{"x": 2, "y": 2}, {"x": 18, "y": 2}], "width": 2}],
        }
        self.assertEqual(validate_canonical_layout(layout), [])
        layout["aisles"][0]["width"] = 0
        self.assertTrue(any("width must be positive" in error for error in validate_canonical_layout(layout)))

    def test_two_point_and_collinear_aisles_are_valid(self):
        for centerline in (
            [{"x": 2, "y": 2}, {"x": 18, "y": 2}],
            [{"x": 2, "y": 2}, {"x": 10, "y": 2}, {"x": 18, "y": 2}],
            [{"x": 2, "y": 2}, {"x": 2, "y": 10}, {"x": 10, "y": 10}],
        ):
            self.assertEqual(validate_aisle_centerline(centerline, 2), [])
        self.assertTrue(validate_aisle_centerline([{"x": 1, "y": 1}], 2))
        self.assertTrue(validate_aisle_centerline([{"x": 1, "y": 1}, {"x": 1, "y": 1}], 2))

    def test_aisle_width_is_contained_and_respects_holes(self):
        base = {
            "size": {"width": 20, "depth": 20},
            "floors": [{"id": 1, "boundary": [[0, 0], [20, 0], [20, 20], [0, 20]], "holes": [[[8, 8], [12, 8], [12, 12], [8, 12]]]}],
        }
        base["aisles"] = [{"id": "inside", "floor_id": 1, "centerline": [{"x": 2, "y": 5}, {"x": 18, "y": 5}], "width": 2}]
        self.assertEqual(validate_canonical_layout(base), [])
        base["aisles"][0]["centerline"] = [{"x": 2, "y": 0.5}, {"x": 18, "y": 0.5}]
        self.assertTrue(any("width footprint" in error for error in validate_canonical_layout(base)))
        base["aisles"][0]["centerline"] = [{"x": 2, "y": 10}, {"x": 18, "y": 10}]
        self.assertTrue(any("width footprint" in error for error in validate_canonical_layout(base)))
        base["aisles"][0]["centerline"] = [{"x": 2, "y": 2}, {"x": 2, "y": 6}, {"x": 6, "y": 6}]
        self.assertEqual(validate_canonical_layout(base), [])
        self.assertEqual(len(aisle_footprint(base["aisles"][0]["centerline"], 2)), 6)

    def test_multi_floor_aisles_keep_their_floor_reference(self):
        layout = {
            "size": {"width": 20, "depth": 20},
            "floors": [{"id": "F1"}, {"id": "F2"}],
            "aisles": [
                {"id": "A-F1", "floor_id": "F1", "centerline": [{"x": 2, "y": 2}, {"x": 18, "y": 2}], "width": 2},
                {"id": "A-F2", "floor_id": "F2", "centerline": [{"x": 2, "y": 4}, {"x": 18, "y": 4}], "width": 2},
            ],
        }
        self.assertEqual(validate_canonical_layout(layout), [])

    def test_navigation_tag_generation_metadata_is_preserved(self):
        layout = {
            "size": {"width": 10, "depth": 10},
            "floors": [{"id": "F1"}],
            "navigation_tags": [{
                "uuid": "tag-F1-A-spacing-5000-1000",
                "tag_id": 1005,
                "floor_id": "F1",
                "x": 5.0,
                "y": 1.0,
                "placement": "manual",
                "locked": True,
                "logical_key": "F1|A|spacing|5000:1000",
                "generated_from": "A",
                "source_aisles": ["A"],
                "semantic_role": "spacing",
                "distance_along_aisle": 5.0,
            }],
        }
        normalized = canonicalize_layout(layout)
        self.assertEqual(normalized["navigation_tags"][0]["logical_key"], "F1|A|spacing|5000:1000")
        self.assertEqual(normalized["navigation_tags"][0]["source_aisles"], ["A"])

    def test_navigation_tags_validate_geometry_identity_and_separation(self):
        base = {"size": {"width": 10, "depth": 10}, "floors": [{"id": "F1", "boundary": [[0, 0], [10, 0], [10, 10], [0, 10]], "holes": [[[4, 4], [6, 4], [6, 6], [4, 6]]]}]}
        base["navigation_tags"] = [{"uuid": "a", "tag_id": 1, "floor_id": "F1", "x": 0.01, "y": 5, "z": 0, "yaw": 0}]
        self.assertEqual(validate_canonical_layout(base), [])
        for changed, needle in [
            ({"x": 11}, "outside floor"), ({"x": 5, "y": 5}, "inside hole"), ({"floor_id": "NO"}, "invalid floor"), ({"uuid": ""}, "missing uuid"), ({"x": float("nan")}, "invalid x/y/z/yaw"),
        ]:
            layout = canonicalize_layout(base); layout["navigation_tags"][0].update(changed)
            self.assertTrue(any(needle in error for error in validate_canonical_layout(layout)))
        duplicate = canonicalize_layout(base); duplicate["navigation_tags"].append({"uuid": "a", "tag_id": 1, "floor_id": "F1", "x": 1, "y": 5, "z": 0, "yaw": 0})
        self.assertTrue(any("duplicate uuid" in error for error in validate_canonical_layout(duplicate)))
        close = canonicalize_layout(base); close["navigation_tags"].append({"uuid": "b", "tag_id": 2, "floor_id": "F1", "x": 0.1, "y": 5, "z": 0, "yaw": 0})
        self.assertTrue(any("minimum separation" in error for error in validate_canonical_layout(close)))

    def test_navigation_edges_validate_identity_floor_direction_and_cost(self):
        layout = {
            "size": {"width": 20, "depth": 10},
            "floors": [{"id": "F1"}],
            "aisles": [{"id": "A", "floor_id": "F1", "centerline": [[1, 5], [19, 5]], "width": 2}],
            "navigation_tags": [
                {"uuid": "t1", "tag_id": 1, "floor_id": "F1", "x": 2, "y": 5, "yaw": 0},
                {"uuid": "t2", "tag_id": 2, "floor_id": "F1", "x": 8, "y": 5, "yaw": 0},
            ],
            "navigation_edges": [{"uuid": "e1", "from_tag_uuid": "t1", "to_tag_uuid": "t2", "aisle_id": "A", "floor_id": "F1", "distance": 6, "cost": 6, "direction": "forward"}],
        }
        self.assertEqual(validate_canonical_layout(layout), [])
        bad = canonicalize_layout(layout)
        bad["navigation_edges"][0]["to_tag_uuid"] = "missing"
        bad["navigation_edges"][0]["cost"] = 0
        self.assertTrue(any("missing tag" in error for error in validate_canonical_layout(bad)))
        self.assertTrue(any("invalid cost" in error for error in validate_canonical_layout(bad)))
