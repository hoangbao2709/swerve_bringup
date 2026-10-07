from asgiref.sync import async_to_sync
from types import SimpleNamespace
import math
import json
from pathlib import Path
from unittest.mock import AsyncMock, patch
import time

from django.test import TestCase
from django.test import Client
from twin.models import NavigationTag, NavigationTagEdge, RobotMapRegistration, RobotNavigationMission, WarehouseMap
from twin.navigation_targets import NavigationTargetError, navigation_tag_registry, resolve_navigation_target
from twin.runtime import runtime
from twin.tag_navigation import shortest_tag_route
from twin.warehouse_services import sync_from_layout
from twin.navigation_graph import (
    generate_orthogonal_edges, plan_orthogonal_tag_route,
    prepare_published_navigation, validate_orthogonal_edges,
    validate_orthogonal_route,
)


class NavigationGraphSyncTests(TestCase):
    def layout(self):
        return {
            'id': 'graph-test', 'name': 'Graph Test', 'units': 'm',
            'size': {'width': 20, 'depth': 10, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [20, 0], [20, 10], [0, 10]]}],
            'aisles': [{'id': 'A', 'floor_id': 'F1', 'centerline': [[2, 5], [14, 5]], 'width': 2.0, 'direction': 'bidirectional'}],
            'navigation_tags': [
                {'uuid': 't1', 'tag_id': 1, 'family': 'APRILTAG', 'size': 0.2, 'lane_id': 'A-01', 'floor_id': 'F1', 'x': 2, 'y': 5, 'z': 0.15, 'yaw': 0},
                {'uuid': 't2', 'tag_id': 2, 'floor_id': 'F1', 'x': 8, 'y': 5, 'yaw': 0},
                {'uuid': 't3', 'tag_id': 3, 'floor_id': 'F1', 'x': 14, 'y': 5, 'yaw': 0},
            ],
            'navigation_edges': [
                {'uuid': 'e12', 'from_tag_uuid': 't1', 'to_tag_uuid': 't2', 'from_tag_id': 1, 'to_tag_id': 2, 'aisle_id': 'A', 'floor_id': 'F1', 'distance': 6, 'cost': 6, 'direction': 'forward', 'enabled': True},
                {'uuid': 'e23', 'from_tag_uuid': 't2', 'to_tag_uuid': 't3', 'from_tag_id': 2, 'to_tag_id': 3, 'aisle_id': 'A', 'floor_id': 'F1', 'distance': 6, 'cost': 6, 'direction': 'forward', 'enabled': True},
            ],
        }

    def test_sync_is_idempotent_and_routing_respects_direction(self):
        layout = self.layout()
        result = sync_from_layout(layout, prune=True)
        self.assertEqual(result['navigation_tags'], 3)
        self.assertEqual(result['navigation_edges'], 2)
        sync_from_layout(layout, prune=True)
        self.assertEqual(NavigationTag.objects.count(), 3)
        self.assertEqual(NavigationTagEdge.objects.count(), 2)
        tag = NavigationTag.objects.get(tag_id=1)
        self.assertEqual(tag.family, 'APRILTAG')
        self.assertAlmostEqual(tag.size, 0.2)
        self.assertEqual(tag.lane_id, 'A-01')
        self.assertAlmostEqual(tag.z, 0.15)
        self.assertEqual(shortest_tag_route(1, 3), [1, 2, 3])
        self.assertEqual(shortest_tag_route(3, 1), [3, 2, 1])

    def test_bidirectional_and_stale_edges(self):
        layout = self.layout()
        layout['navigation_edges'][0]['direction'] = 'bidirectional'
        layout['navigation_edges'][0]['bidirectional'] = True
        sync_from_layout(layout, prune=True)
        self.assertEqual(shortest_tag_route(2, 1), [2, 1])
        layout['navigation_edges'] = layout['navigation_edges'][:1]
        sync_from_layout(layout, prune=True)
        # A published aisle remains the authority for adjacent edges even if
        # the stale serialized edge list is shortened.
        self.assertEqual(NavigationTagEdge.objects.count(), 2)
        self.assertEqual(shortest_tag_route(3, 1), [3, 2, 1])


class OrthogonalTagRouteTests(TestCase):
    def layout(self):
        tags = [
            {'uuid': 'a', 'tag_id': 1, 'floor_id': 'F1', 'x': 10, 'y': 5, 'yaw': 0,
             'source_aisles': ['H1'], 'semantic_role': 'intersection'},
            {'uuid': 'b', 'tag_id': 2, 'floor_id': 'F1', 'x': 15, 'y': 5, 'yaw': 0,
             'source_aisles': ['H1', 'V2'], 'semantic_role': 'intersection'},
            {'uuid': 'c', 'tag_id': 3, 'floor_id': 'F1', 'x': 15, 'y': 15, 'yaw': 0,
             'source_aisles': ['V2', 'H2'], 'semantic_role': 'intersection'},
            {'uuid': 'd', 'tag_id': 4, 'floor_id': 'F1', 'x': 20, 'y': 15, 'yaw': 0,
             'source_aisles': ['H2'], 'semantic_role': 'intersection'},
        ]
        return {
            'id': 'orthogonal-route-test', 'size': {'width': 30, 'depth': 25, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [30, 0], [30, 25], [0, 25]]}],
            'aisles': [
                {'id': 'H1', 'floor_id': 'F1', 'centerline': [[10, 5], [15, 5]], 'width': 1.5, 'direction': 'bidirectional'},
                {'id': 'V2', 'floor_id': 'F1', 'centerline': [[15, 5], [15, 15]], 'width': 1.5, 'direction': 'bidirectional'},
                {'id': 'H2', 'floor_id': 'F1', 'centerline': [[15, 15], [20, 15]], 'width': 1.5, 'direction': 'bidirectional'},
            ],
            'navigation_tags': tags, 'navigation_edges': [],
            'racks': [], 'stations': [], 'obstacles': [], 'columns': [], 'conveyors': [],
        }

    def test_generated_graph_and_route_are_orthogonal_and_visit_intermediate_nodes(self):
        layout = self.layout()
        edges = generate_orthogonal_edges(layout)
        self.assertEqual([(edge['from_tag_id'], edge['to_tag_id'], edge['axis']) for edge in edges],
                         [(1, 2, 'X'), (2, 3, 'Y'), (3, 4, 'X')])
        self.assertFalse(validate_orthogonal_edges({**layout, 'navigation_edges': edges}))
        self.assertFalse(any({edge['from_tag_id'], edge['to_tag_id']} == {1, 3} for edge in edges))
        route = plan_orthogonal_tag_route(layout, {'x': 10, 'y': 5, 'yaw': 0}, 4)
        self.assertEqual(route['route_nodes'], [1, 2, 3, 4])
        self.assertEqual([segment['axis'] for segment in route['route_segments']], ['X', 'Y', 'X'])
        self.assertTrue(validate_orthogonal_route(route['route_points']))
        self.assertIn('X', {segment['axis'] for segment in route['route_segments']})
        self.assertIn('Y', {segment['axis'] for segment in route['route_segments']})

    def test_nearby_graph_entry_uses_tag_pose_instead_of_an_arbitrary_micro_goal(self):
        route = plan_orthogonal_tag_route(
            self.layout(), {'x': 10.06, 'y': 5.01, 'yaw': -0.2}, 4)
        self.assertEqual(route['route_nodes'], [1, 2, 3, 4])
        self.assertEqual(route['route_points'][1]['kind'], 'TAG')
        self.assertEqual(route['route_points'][1]['tag_id'], 1)
        self.assertAlmostEqual(route['route_points'][1]['yaw'], 0.0)
        self.assertEqual(route['route_segments'][0]['from'], 'ROBOT_START')
        self.assertEqual(route['route_segments'][0]['to'], 1)
        self.assertTrue(validate_orthogonal_route(route['route_points']))

    def test_short_but_real_l_shaped_entry_connector_remains_explicit(self):
        route = plan_orthogonal_tag_route(
            self.layout(), {'x': 15.1, 'y': 5.1, 'yaw': 1.1}, 4)
        self.assertEqual(route['route_nodes'], [2, 3, 4])
        self.assertEqual([point['kind'] for point in route['route_points'][:4]],
                         ['START', 'LANE_CONNECTOR', 'LANE_CONNECTOR', 'TAG'])
        self.assertEqual([segment['axis'] for segment in route['route_segments'][:2]], ['Y', 'X'])
        self.assertAlmostEqual(route['route_points'][1]['yaw'], math.pi / 2)
        self.assertAlmostEqual(route['route_points'][2]['yaw'], math.pi / 2)
        self.assertAlmostEqual(route['route_points'][3]['yaw'], math.pi / 2)
        self.assertAlmostEqual(route['route_points'][-1]['yaw'], 0.0)
        self.assertTrue(validate_orthogonal_route(route['route_points']))

    def test_transit_tags_keep_one_cardinal_body_heading_across_x_and_y_legs(self):
        route = plan_orthogonal_tag_route(
            self.layout(), {'x': 10.0, 'y': 5.0, 'yaw': math.pi / 2}, 4)
        self.assertEqual([segment['axis'] for segment in route['route_segments']], ['X', 'Y', 'X'])
        for point in route['route_points'][1:-1]:
            self.assertAlmostEqual(point['yaw'], math.pi / 2)
        # Transit heading is lane-oriented, while the destination's authored
        # service pose remains authoritative.
        self.assertAlmostEqual(route['route_points'][-1]['yaw'], 0.0)
        self.assertTrue(validate_orthogonal_route(route['route_points']))

    def test_shelf_service_chooses_equivalent_heading_matching_transit_without_breaking_width_alignment(self):
        for rack_yaw, transit_yaw in ((0, math.pi / 2), (90, 0.0), (30, 0.0),
                                      (180, -math.pi / 2), (-180, math.pi / 2)):
            with self.subTest(rack_yaw=rack_yaw, transit_yaw=transit_yaw):
                layout = self._rotated_shelf_layout(rack_yaw)
                tag = prepare_published_navigation(layout)['navigation_tags'][0]
                route = plan_orthogonal_tag_route(
                    layout, {'x': tag['x'], 'y': tag['y'], 'yaw': transit_yaw}, 7)
                final_yaw = route['destination_pose']['yaw']
                rack_long_axis = math.radians(rack_yaw)
                width_axis_yaw = final_yaw + math.pi / 2
                alignment_error = abs(math.atan2(math.sin(width_axis_yaw - rack_long_axis),
                                                 math.cos(width_axis_yaw - rack_long_axis)))
                alignment_error = min(alignment_error, abs(math.pi - alignment_error))
                self.assertEqual(route['orientation_policy'], 'SHELF_WIDTH_PARALLEL')
                self.assertLess(alignment_error, 1e-8)
                self.assertLess(abs(math.atan2(math.sin(final_yaw - transit_yaw),
                                               math.cos(final_yaw - transit_yaw))), math.pi / 2 + 1e-8)
                self.assertAlmostEqual(route['route_points'][-1]['yaw'], final_yaw)

    def test_route_graph_revision_uses_active_published_revision_not_stale_layout_revision(self):
        layout = self.layout()
        layout['revision'] = 21
        expected = prepare_published_navigation(layout, canonical_revision=22)['tag_graph_revision']
        route = plan_orthogonal_tag_route(
            layout, {'x': 10, 'y': 5, 'yaw': 0}, 4, canonical_revision=22)
        stale = prepare_published_navigation(layout)['tag_graph_revision']
        self.assertEqual(route['graph_revision'], expected)
        self.assertNotEqual(route['graph_revision'], stale)

    def test_complete_published_tag_graph_has_no_diagonal_edges(self):
        repository = Path(__file__).resolve().parents[4]
        source = repository / 'generated/maps/WH-TEST-01/22/canonical_map.json'
        published = json.loads(source.read_text())
        graph = prepare_published_navigation(published, 22)
        self.assertEqual(len(graph['navigation_edges']), 47)
        self.assertFalse(validate_orthogonal_edges(graph))
        self.assertTrue(all(edge['axis'] in ('X', 'Y') for edge in graph['navigation_edges']))

    def test_published_warehouse_shelf_semantics_are_geometry_derived_and_revision_bound(self):
        repository = Path(__file__).resolve().parents[4]
        source = repository / 'generated/maps/WH-TEST-01/22/canonical_map.json'
        published = json.loads(source.read_text())
        graph = prepare_published_navigation(published, 23)
        shelf_tags = [tag for tag in graph['navigation_tags']
                      if tag.get('semantic_role') == 'shelf_service']
        lane_tags = [tag for tag in graph['navigation_tags'] if tag not in shelf_tags]

        # This bundle deliberately has rack-aligned outer service lanes plus a
        # middle navigation lane.  The classification is checked by geometry
        # and exact rack association, not Tag-number prefixes.
        self.assertEqual(len(shelf_tags), 20)
        self.assertEqual(len(lane_tags), 10)
        self.assertEqual(len(graph['navigation_edges']), 47)
        self.assertFalse(validate_orthogonal_edges(graph))
        racks = {str(rack['id']): rack for rack in published['racks']}
        for tag in shelf_tags:
            metadata = tag['metadata']
            self.assertEqual(metadata['orientation_policy'], 'SHELF_WIDTH_PARALLEL')
            rack = racks[metadata['rack_id']]
            rack_yaw = math.radians(float(rack.get('rotation') or 0.0))
            if float(rack['size'][2]) > float(rack['size'][0]):
                rack_yaw += math.pi / 2
            width_yaw = tag['yaw'] + math.pi / 2
            parallel_error = abs(math.atan2(math.sin(width_yaw - rack_yaw),
                                            math.cos(width_yaw - rack_yaw)))
            parallel_error = min(parallel_error, abs(math.pi - parallel_error))
            self.assertLess(parallel_error, 1e-8)
            self.assertEqual(metadata['service_pose']['frame_id'], 'map')
            self.assertEqual(metadata['service_pose']['map_revision'], '23')
            self.assertIn(metadata['service_face'],
                          ('LONG_AXIS_POSITIVE_END', 'LONG_AXIS_NEGATIVE_END'))
        self.assertTrue(all(tag['metadata']['orientation_policy'] == 'LANE_FORWARD'
                            for tag in lane_tags))

    @staticmethod
    def _rotated_shelf_layout(rack_yaw):
        cx, cy = 12.0, 11.0
        long_axis_yaw = math.radians(rack_yaw)
        tag_x = cx + 3.0 * math.cos(long_axis_yaw)
        tag_y = cy + 3.0 * math.sin(long_axis_yaw)
        lane_end = (tag_x + 6.0 * math.cos(long_axis_yaw),
                    tag_y + 6.0 * math.sin(long_axis_yaw))
        return {
            'id': 'rotated-rack', 'size': {'width': 30, 'depth': 30, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [30, 0], [30, 30], [0, 30]]}],
            'aisles': [{'id': 'service-lane', 'floor_id': 'F1',
                        'centerline': [[tag_x, tag_y], list(lane_end)],
                        'width': 1.8, 'direction': 'bidirectional'}],
            'navigation_edges': [], 'stations': [], 'obstacles': [], 'columns': [],
            'conveyors': [],
            'racks': [{'id': 'rack-A', 'floor': 'F1', 'position': [10, 0, 10],
                       'size': [4, 2, 2], 'rotation': rack_yaw}],
            'navigation_tags': [{'uuid': 'shelf-tag', 'tag_id': 7, 'floor_id': 'F1',
                'x': tag_x, 'y': tag_y, 'yaw': 0, 'source_aisles': ['service-lane'],
                'semantic_role': 'shelf_service', 'metadata': {'rack_id': 'rack-A'}}],
        }

    def test_shelf_width_axis_uses_actual_rotated_rack_long_axis(self):
        for rack_yaw in (0, 90, 30, 180, -180):
            with self.subTest(rack_yaw=rack_yaw):
                tag = prepare_published_navigation(
                    self._rotated_shelf_layout(rack_yaw))['navigation_tags'][0]
                rack_long_axis = math.radians(rack_yaw)
                robot_width_axis = tag['yaw'] + math.pi / 2
                error = abs(math.atan2(math.sin(robot_width_axis - rack_long_axis),
                                       math.cos(robot_width_axis - rack_long_axis)))
                error = min(error, abs(math.pi - error))
                self.assertEqual(tag['metadata']['orientation_policy'], 'SHELF_WIDTH_PARALLEL')
                self.assertLess(error, 1e-9)

    def test_authored_non_shelf_policy_is_not_promoted_by_nearby_rack_geometry(self):
        layout = self._rotated_shelf_layout(30)
        tag = layout['navigation_tags'][0]
        tag['semantic_role'] = 'intersection'
        tag['metadata'] = {'orientation_policy': 'AXIS_Y_PARALLEL'}
        prepared = prepare_published_navigation(layout)['navigation_tags'][0]
        self.assertEqual(prepared['semantic_role'], 'intersection')
        self.assertNotIn('rack_id', prepared['metadata'])
        self.assertEqual(prepared['metadata']['orientation_policy'], 'AXIS_Y_PARALLEL')
        self.assertAlmostEqual(prepared['yaw'], math.pi / 2)

    def test_ambiguous_shelf_service_requires_explicit_rack_and_bad_association_fails(self):
        layout = self._rotated_shelf_layout(0)
        layout['racks'].append({**layout['racks'][0], 'id': 'rack-B'})
        layout['navigation_tags'][0]['metadata'] = {}
        with self.assertRaisesRegex(ValueError, 'ambiguous shelf-service geometry'):
            prepare_published_navigation(layout)

        layout['navigation_tags'][0]['metadata'] = {'rack_id': 'missing-rack'}
        with self.assertRaisesRegex(ValueError, 'references missing rack_id'):
            prepare_published_navigation(layout)

    def test_non_shelf_axis_orientation_policy_is_cardinal(self):
        layout = self.layout()
        layout['navigation_tags'][0]['metadata'] = {'orientation_policy': 'AXIS_Y_PARALLEL'}
        tag = prepare_published_navigation(layout)['navigation_tags'][0]
        allowed = (0.0, math.pi / 2, math.pi, -math.pi / 2)
        self.assertEqual(tag['metadata']['orientation_policy'], 'AXIS_Y_PARALLEL')
        self.assertLess(min(abs(math.atan2(math.sin(tag['yaw'] - yaw),
                                      math.cos(tag['yaw'] - yaw))) for yaw in allowed), 1e-9)


class NavigationTargetResolutionTests(TestCase):
    def setUp(self):
        self.layout = {
            'id': 'target-resolution-test', 'name': 'Target Resolution Test', 'units': 'm',
            'size': {'width': 20, 'depth': 10, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [20, 0], [20, 10], [0, 10]]}],
            'aisles': [{'id': 'A-01', 'floor_id': 'F1',
                        'centerline': [[2.0, 5.0], [8.0, 5.0]],
                        'width': 1.5, 'direction': 'bidirectional'}],
            'navigation_tags': [
                {'uuid': 't1', 'tag_id': 1, 'family': 'APRILTAG', 'label': 'Aisle node',
                 'floor_id': 'F1', 'lane_id': 'A-01', 'x': 2.0, 'y': 5.0, 'z': 0.0, 'yaw': 0.25},
                {'uuid': 't2', 'tag_id': 2, 'label': 'Disabled node',
                 'floor_id': 'F1', 'x': 8.0, 'y': 5.0, 'yaw': 0.0, 'enabled': False},
            ],
            'navigation_edges': [],
        }
        result = sync_from_layout(self.layout, prune=True)
        WarehouseMap.objects.update(is_active=False)
        self.warehouse_map = WarehouseMap.objects.create(
            warehouse_id=result['warehouse_id'], layout=self.layout, draft=self.layout,
            revision=7, published_version=1, is_active=True,
        )
        self.active_map = {
            'active_map_id': 'CANONICAL', 'active_map_revision': '7',
            'canonical_map_revision': 7, 'map_sync_status': 'CANONICAL',
        }

    def test_tag_graph_endpoint_uses_the_active_published_geometry_and_revision(self):
        response = Client().get('/api/navigation/tag-graph')
        self.assertEqual(response.status_code, 200)
        payload = response.json()
        expected = prepare_published_navigation(self.layout, canonical_revision=7)
        self.assertEqual(payload['canonical_revision'], 7)
        self.assertEqual(payload['graph_revision'], expected['tag_graph_revision'])
        self.assertEqual(payload['tags'], expected['navigation_tags'])
        self.assertEqual(payload['edges'], expected['navigation_edges'])
        self.assertFalse(validate_orthogonal_edges(
            {**self.layout, 'navigation_edges': payload['edges']}, payload['edges']))

    def test_shelf_tag_resolution_preserves_both_valid_width_parallel_directions(self):
        from twin.navigation_graph import prepare_published_navigation

        layout = OrthogonalTagRouteTests._rotated_shelf_layout(0)
        prepared = prepare_published_navigation(layout, canonical_revision=7)
        result = sync_from_layout(prepared, prune=True)
        WarehouseMap.objects.update(is_active=False)
        warehouse_map = WarehouseMap.objects.create(
            warehouse_id=result['warehouse_id'], layout=prepared, draft=prepared,
            revision=7, published_version=1, is_active=True,
        )
        active_map = {'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                      'canonical_map_revision': 7, 'map_sync_status': 'CANONICAL'}
        base = resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=active_map, tag_id=7)
        preferred = resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=active_map, tag_id=7,
            preferred_yaw=math.pi / 2)
        repeated = resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=active_map, tag_id=7,
            preferred_yaw=preferred['yaw'])
        self.assertEqual(base['metadata']['orientation_policy'], 'SHELF_WIDTH_PARALLEL')
        self.assertAlmostEqual(base['x'], preferred['x'])
        self.assertAlmostEqual(base['y'], preferred['y'])
        self.assertAlmostEqual(preferred['yaw'], math.pi / 2)
        self.assertAlmostEqual(repeated['yaw'], preferred['yaw'])
        self.assertEqual(base['metadata']['tag_revision'], preferred['metadata']['tag_revision'])
        self.assertEqual(warehouse_map.revision, 7)

    def resolve(self, tag_id, active_map=None):
        return resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=active_map or self.active_map,
            tag_id=tag_id,
        )

    def test_tag_route_leg_authorization_rechecks_current_graph_and_tag_revision(self):
        registry = navigation_tag_registry(self.active_map, robot_id='R01')
        tag = next(item for item in registry['tags'] if item['tag_id'] == 1)
        graph = prepare_published_navigation(self.warehouse_map.layout, 7)
        route_plan = {
            'graph_revision': graph['tag_graph_revision'],
            'route_nodes': [1],
            'active_route_points': [
                {'x': 0.0, 'y': 5.0, 'yaw': 0.0, 'tag_id': None},
                {'x': 2.0, 'y': 5.0, 'yaw': 0.25, 'tag_id': 1},
            ],
            'route_tag_revisions': {'1': tag['tag_revision']},
        }
        execution = {
            'route_revision': 'approved-route-revision', 'route_plan': route_plan,
            'registry_revision': registry['registry_revision'],
            'registration_revision': None, 'registration_source': None,
            'active_map_id': 'CANONICAL', 'active_map_revision': '7',
            'canonical_map_revision': 7, 'navigation_map_revision': None,
        }
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        request = {
            'robot_id': 'R01', 'auth_request_id': 'auth-1',
            'route_revision': 'approved-route-revision',
            'graph_revision': graph['tag_graph_revision'], 'route_nodes': [1],
            'route_index': 1, 'route_node_id': 1,
        }

        with patch.object(runtime, 'tag_route_executions', {'R01': execution}), \
                patch.object(runtime, 'robot_runtime_modes', {'R01': 'NAVIGATION'}), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'robot_navigation_maps', {}), \
                patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'navigation_goal_blocker', return_value=None), \
                patch.object(runtime, 'gateway', return_value=gateway):
            async_to_sync(runtime.handle_tag_route_leg_auth_request)(request)

        gateway.send_command.assert_awaited_once()
        self.assertEqual(gateway.send_command.await_args.args[:2], ('R01', 'TAG_ROUTE_LEG_AUTH'))
        self.assertTrue(gateway.send_command.await_args.args[2]['approved'])
        self.assertEqual(gateway.send_command.await_args.args[2]['route_index'], 1)

        stale_plan = {**route_plan, 'route_tag_revisions': {'1': 'old-tag-revision'}}
        stale_execution = {**execution, 'route_plan': stale_plan}
        gateway.send_command.reset_mock()
        with patch.object(runtime, 'tag_route_executions', {'R01': stale_execution}), \
                patch.object(runtime, 'robot_runtime_modes', {'R01': 'NAVIGATION'}), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'robot_navigation_maps', {}), \
                patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'navigation_goal_blocker', return_value=None), \
                patch.object(runtime, 'gateway', return_value=gateway):
            async_to_sync(runtime.handle_tag_route_leg_auth_request)(request)
        self.assertFalse(gateway.send_command.await_args.args[2]['approved'])
        self.assertEqual(gateway.send_command.await_args.args[2]['reason_code'], 'TAG_REVISION_MISMATCH')

    def test_valid_tag_resolves_registered_navigation_node_with_registry_hash(self):
        target = self.resolve(1)
        self.assertEqual(target['robot_id'], 'R01')
        self.assertEqual(target['source_type'], 'TAG')
        self.assertEqual(target['source_id'], 1)
        self.assertEqual((target['frame_id'], target['map_id'], target['map_revision']), ('map', 'CANONICAL', '7'))
        self.assertEqual((target['x'], target['y'], target['yaw']), (2.0, 5.0, 0.25))
        self.assertEqual(target['metadata']['pose_source'], 'REGISTERED_NAVIGATION_NODE')
        self.assertEqual(len(target['metadata']['tag_revision']), 64)
        self.assertEqual(len(target['metadata']['registry_revision']), 64)

    def test_map_point_and_tag_share_the_same_navigation_target_shape(self):
        point = resolve_navigation_target(
            robot_id='R01', source_type='MAP_POINT', active_map=self.active_map,
            x=4.0, y=6.0, yaw=0.5,
        )
        tag = self.resolve(1)
        self.assertEqual(set(point), set(tag))
        self.assertEqual((point['frame_id'], point['map_id'], point['map_revision']),
                         (tag['frame_id'], tag['map_id'], tag['map_revision']))
        self.assertEqual((point['source_type'], point['source_id']), ('ACTIVE_MAP_POINT', None))

    def test_map_point_resolves_in_the_active_local_map_without_coordinate_reinterpretation(self):
        local_map = {**self.active_map, 'active_map_id': 'local-R01-map',
                     'active_map_revision': 'local-rev-3', 'map_sync_status': 'LOCAL_ONLY'}
        point = resolve_navigation_target(robot_id='R01', source_type='MAP_POINT', active_map=local_map,
            x=1.25, y=-0.5, yaw=0.3)
        self.assertEqual((point['frame_id'], point['map_id'], point['map_revision']),
                         ('map', 'local-R01-map', 'local-rev-3'))
        self.assertEqual((point['x'], point['y'], point['yaw']), (1.25, -0.5, 0.3))

    def test_explicit_approach_pose_is_used_without_an_invented_offset(self):
        tag = NavigationTag.objects.get(warehouse=self.warehouse_map.warehouse, tag_id=1)
        tag.metadata = {'approach_pose': {'x': 1.5, 'y': 4.0, 'yaw': -0.2, 'frame_id': 'map', 'map_revision': '7'}}
        tag.save(update_fields=['metadata', 'updated_at'])
        target = self.resolve(1)
        self.assertEqual((target['x'], target['y'], target['yaw']), (1.5, 4.0, -0.2))
        self.assertEqual(target['metadata']['pose_source'], 'METADATA:approach_pose')

    def test_non_finite_registered_navigation_pose_is_not_navigable(self):
        tag = NavigationTag.objects.get(warehouse=self.warehouse_map.warehouse, tag_id=1)
        tag.metadata = {'navigation_pose': {'x': 'nan', 'y': 4.0, 'yaw': 0.0}}
        tag.save(update_fields=['metadata', 'updated_at'])
        registry = navigation_tag_registry(self.active_map)
        self.assertFalse(next(item for item in registry['tags'] if item['tag_id'] == 1)['navigable'])
        with self.assertRaises(NavigationTargetError) as invalid:
            self.resolve(1)
        self.assertEqual(invalid.exception.code, 'TAG_NOT_NAVIGABLE')

    def test_unknown_disabled_wrong_map_and_stale_revision_tags_are_rejected(self):
        with self.assertRaises(NavigationTargetError) as unknown:
            self.resolve(999)
        self.assertEqual(unknown.exception.code, 'TAG_UNKNOWN')

        with self.assertRaises(NavigationTargetError) as disabled:
            self.resolve(2)
        self.assertEqual(disabled.exception.code, 'TAG_DISABLED')

        local_map = {**self.active_map, 'active_map_id': 'local-R01-map', 'map_sync_status': 'LOCAL_ONLY'}
        with self.assertRaises(NavigationTargetError) as wrong_map:
            self.resolve(1, local_map)
        self.assertEqual(wrong_map.exception.code, 'TAG_MAP_REGISTRATION_REQUIRED')

        stale_map = {**self.active_map, 'active_map_revision': '6'}
        with self.assertRaises(NavigationTargetError) as stale:
            self.resolve(1, stale_map)
        self.assertEqual(stale.exception.code, 'TAG_MAP_REVISION_MISMATCH')

    def test_local_map_tag_registry_is_informational_until_versioned_registration_exists(self):
        local_map = {**self.active_map, 'active_map_id': 'local-R01-map',
                     'active_map_revision': 'local-rev-3', 'map_sync_status': 'LOCAL_ONLY'}
        registry = navigation_tag_registry(local_map)
        self.assertFalse(registry['compatible'])
        self.assertEqual(registry['reason'], 'TAG_MAP_REGISTRATION_REQUIRED')
        self.assertTrue(registry['registration_required'])
        self.assertEqual((registry['map_id'], registry['map_revision']), ('CANONICAL', '7'))
        self.assertEqual((registry['active_map_id'], registry['active_map_revision']),
                         ('local-R01-map', 'local-rev-3'))
        self.assertEqual([tag['tag_id'] for tag in registry['tags']], [1, 2])
        self.assertTrue(all(not tag['navigable'] and tag['reason'] == 'TAG_MAP_REGISTRATION_REQUIRED'
                            and tag['map_id'] == 'CANONICAL' and tag['map_revision'] == '7'
                            for tag in registry['tags']))
        with self.assertRaises(NavigationTargetError) as blocked:
            self.resolve(1, local_map)
        self.assertEqual(blocked.exception.code, 'TAG_MAP_REGISTRATION_REQUIRED')

    def test_registered_local_map_transforms_tags_with_revision_bound_se2(self):
        local_map = {**self.active_map, 'active_map_id': 'SLAM-session-4',
                     'active_map_revision': 'slam-rev-12', 'map_sync_status': 'LOCAL_ONLY'}
        RobotMapRegistration.objects.create(
            robot_id='R01', warehouse_map=self.warehouse_map, canonical_revision=7,
            active_map_id='SLAM-session-4', active_map_revision='slam-rev-12',
            tx=10.0, ty=-2.0, yaw=math.pi / 2,
            registration_revision=3, source='survey-control-points',
        )

        registry = navigation_tag_registry(local_map, robot_id='R01')
        self.assertTrue(registry['compatible'])
        self.assertFalse(registry['registration_required'])
        self.assertEqual(registry['transform_source'], 'survey-control-points')
        self.assertEqual(registry['registration_revision'], 3)
        tag = next(item for item in registry['tags'] if item['tag_id'] == 1)
        self.assertTrue(tag['navigable'])
        self.assertEqual((tag['map_id'], tag['map_revision']),
                         ('SLAM-session-4', 'slam-rev-12'))
        self.assertEqual(tag['canonical_navigation_pose'], {'x': 2.0, 'y': 5.0, 'yaw': 0.25})
        self.assertEqual(tag['navigation_pose']['x'], 5.0)
        self.assertAlmostEqual(tag['navigation_pose']['y'], 0.0)
        self.assertAlmostEqual(tag['navigation_pose']['yaw'], math.pi / 2 + 0.25)

        target = resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=local_map, tag_id=1)
        self.assertEqual((target['map_id'], target['map_revision']),
                         ('SLAM-session-4', 'slam-rev-12'))
        self.assertAlmostEqual(target['x'], 5.0)
        self.assertAlmostEqual(target['y'], 0.0)
        self.assertEqual(target['metadata']['registration_revision'], 3)
        self.assertEqual(target['metadata']['registration_source'], 'survey-control-points')

        stale_map = {**local_map, 'active_map_revision': 'slam-rev-13'}
        stale_registry = navigation_tag_registry(stale_map, robot_id='R01')
        self.assertFalse(stale_registry['compatible'])
        self.assertEqual(stale_registry['reason'], 'TAG_MAP_REGISTRATION_REQUIRED')
        with self.assertRaises(NavigationTargetError) as stale:
            resolve_navigation_target(robot_id='R01', source_type='TAG',
                                      active_map=stale_map, tag_id=1)
        self.assertEqual(stale.exception.code, 'TAG_MAP_REGISTRATION_REQUIRED')

    def test_local_caller_can_register_exact_active_map_and_registration_is_versioned(self):
        client = Client()
        local_map = {**self.active_map, 'active_map_id': 'SLAM-session-4',
                     'active_map_revision': 'slam-rev-12', 'map_sync_status': 'LOCAL_ONLY'}
        payload = {
            'canonical_map_revision': 7,
            'active_map_id': 'SLAM-session-4', 'active_map_revision': 'slam-rev-12',
            'tx': 10, 'ty': -2, 'yaw': math.pi / 2, 'source': 'survey-control-points',
        }
        with patch.object(runtime, 'active_map_state', return_value=local_map):
            first = client.post('/api/robots/R01/map-registration', data=payload,
                                content_type='application/json')
            self.assertEqual(first.status_code, 201, first.content)
            self.assertEqual(first.json()['registration_revision'], 1)
            updated = client.post('/api/robots/R01/map-registration',
                data={**payload, 'tx': 11}, content_type='application/json')
            self.assertEqual(updated.status_code, 201, updated.content)
            self.assertEqual(updated.json()['registration_revision'], 2)

            tags = client.get('/api/robots/R01/navigation-tags')

        self.assertTrue(tags.json()['compatible'])
        self.assertEqual(tags.json()['registration_revision'], 2)
        active_records = RobotMapRegistration.objects.filter(
            robot_id='R01', active_map_id='SLAM-session-4', is_active=True)
        self.assertEqual(active_records.count(), 1)
        self.assertEqual(active_records.get().tx, 11)

        with patch.object(runtime, 'active_map_state', return_value=local_map):
            stale_identity = client.post('/api/robots/R01/map-registration',
                data={**payload, 'active_map_revision': 'stale-revision'},
                content_type='application/json')
        self.assertEqual(stale_identity.status_code, 409)
        self.assertEqual(stale_identity.json()['error']['details']['code'],
                         'ACTIVE_MAP_REVISION_MISMATCH')

    def test_external_legacy_tag_missions_cannot_bypass_preview_and_nav2_goal(self):
        client = Client()
        tag = NavigationTag.objects.get(warehouse=self.warehouse_map.warehouse, tag_id=1)
        mission = RobotNavigationMission.objects.create(
            warehouse=self.warehouse_map.warehouse, robot_id='R01', target_tag=tag,
            current_tag_id=1, next_tag_id=1, route=[1], status='PAUSED')
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        tag_command = AsyncMock(return_value={'ok': True})

        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime, 'tag_command', tag_command):
            started = client.post('/api/navigation/missions/start',
                data={'robot_id': 'R01', 'target_tag_id': 1}, content_type='application/json')
            resumed = client.post(f'/api/navigation/missions/{mission.pk}/resume', data='{}',
                                  content_type='application/json')
            replanned = client.post(f'/api/navigation/missions/{mission.pk}/replan', data='{}',
                                    content_type='application/json')

        for response in (started, resumed, replanned):
            self.assertEqual(response.status_code, 409)
            self.assertEqual(response.json()['error']['details']['code'],
                             'TAG_NAVIGATION_SINGLE_PIPELINE_REQUIRED')
        tag_command.assert_not_awaited()
        gateway.send_command.assert_not_awaited()
        mission.refresh_from_db()
        self.assertEqual(mission.status, 'PAUSED')

    def test_robot_tag_api_works_without_identity_and_returns_only_map_compatible_registry(self):
        client = Client()
        with patch.object(runtime, 'active_map_state', return_value=self.active_map):
            response = client.get('/api/robots/R01/navigation-tags')
        self.assertEqual(response.status_code, 200)
        body = response.json()
        self.assertEqual(body['source'], 'WAREHOUSE_NAVIGATION_TAG_REGISTRY')
        self.assertTrue(body['compatible'])
        self.assertEqual(body['map_revision'], '7')
        self.assertEqual([tag['tag_id'] for tag in body['tags']], [1, 2])
        self.assertEqual(sum(tag['navigable'] for tag in body['tags']), 1)

        wrong_map = {**self.active_map, 'active_map_id': 'local-R01-map', 'map_sync_status': 'LOCAL_ONLY'}
        with patch.object(runtime, 'active_map_state', return_value=wrong_map):
            response = client.get('/api/robots/R01/navigation-tags')
        self.assertFalse(response.json()['compatible'])
        self.assertEqual(response.json()['reason'], 'TAG_MAP_REGISTRATION_REQUIRED')
        self.assertEqual([tag['tag_id'] for tag in response.json()['tags']], [1, 2])
        self.assertTrue(all(not tag['navigable'] for tag in response.json()['tags']))

        with patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'operation_mode', 'MAPPING'):
            response = client.get('/api/robots/R01/navigation-tags')
        self.assertTrue(response.json()['compatible'])
        self.assertEqual(response.json()['active_map_id'], 'CANONICAL')

        anonymous = Client().get('/api/robots/R01/navigation-tags')
        self.assertEqual(anonymous.status_code, 200)

    def test_tag_preview_uses_common_nav2_pipeline_and_rejects_stale_or_wrong_source_tokens(self):
        registry = navigation_tag_registry(self.active_map)
        tag_record = next(item for item in registry['tags'] if item['tag_id'] == 1)
        capture = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        localization_state = {'frame_id': 'map', 'map_id': 'CANONICAL', 'map_revision': '7',
                              'map_source': 'CANONICAL', 'pose_source': 'TF'}
        request_counter = 0

        def request_preview(*, tag_revision=None, registry_revision=None):
            nonlocal request_counter
            request_counter += 1
            request_id = f'tag-preview-{request_counter}'
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01', 'request_id': request_id,
                'source_type': 'TAG', 'tag_id': 1,
                'tag_revision': tag_revision or tag_record['tag_revision'],
                'registry_revision': registry_revision or registry['registry_revision'], 'frame_id': 'map',
                'active_map_id': 'CANONICAL', 'active_map_revision': '7',
            })
            return request_id

        def planner_result(request_id, goal=(2.0, 5.0, 0.25)):
            preview_payload = gateway.send_command.await_args.args[2]
            async_to_sync(runtime.handle_ros_message)({
                'type': 'PATH_PREVIEW_RESULT', 'robot_id': 'R01', 'request_id': request_id,
                'status': 'VALID', 'path': [[0.0, 0.0], [goal[0], goal[1]]],
                'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]},
                'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                'route_revision': preview_payload['route_revision'],
                'route_nodes': preview_payload['route_nodes'],
                'route_segments': preview_payload['route_segments'],
                'route_points': preview_payload['route_points'],
                'canonical_route_points': preview_payload['canonical_route_points'],
                'route_tag_revisions': preview_payload['route_tag_revisions'],
            })

        def send_goal(request_id, source_id='1', source_type='TAG', goal=(2.0, 5.0, 0.25)):
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': goal[0], 'y': goal[1], 'yaw': goal[2],
                'frame_id': 'map', 'preview_request_id': request_id,
                'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                'source_type': source_type, 'source_id': source_id,
                'source_map_id': 'CANONICAL', 'source_map_revision': '7',
                'route_revision': runtime.path_preview_results.get(
                    ('R01', request_id), {}).get('route_revision'),
            })

        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'unified_navigation_blocker', return_value=None), \
                patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'robot_bridge_online', return_value=True), \
                patch.object(runtime, 'navigation_localization_state', return_value=localization_state), \
                patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime, 'broadcast', new=AsyncMock()), \
                patch.object(runtime, 'path_preview_requests', {}), \
                patch.object(runtime, 'approved_path_previews', {}), \
                patch.object(runtime, 'path_preview_results', {}), \
                patch.object(runtime, 'expired_path_previews', {}), \
                patch.object(runtime, 'path_preview_invalidations', {}), \
                patch.object(runtime.engine, 'state', {'robots': {'R01': {
                    'active_map_pose': {'valid': True, 'frame_id': 'map',
                        'map_id': 'CANONICAL', 'map_revision': '7', 'map_source': 'CANONICAL',
                        'pose_source': 'TF', 'x': 2.0, 'y': 5.0, 'yaw': 0.0},
                }}}), \
                patch.object(runtime, 'robot_pose_heartbeats', {'R01': time.monotonic()}):
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 2.0, 'y': 5.0, 'yaw': 0.25,
                'frame_id': 'map', 'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                'source_type': 'TAG', 'source_id': '1',
            })
            self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_REQUIRED')
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            gateway.send_command.reset_mock()
            stale_request = request_preview(tag_revision='stale-tag-revision')
            self.assertEqual(capture.send_json.await_args.args[0]['status'], 'INVALID')
            self.assertIn('TAG_REVISION_MISMATCH', capture.send_json.await_args.args[0]['reason'])
            self.assertFalse(any(call.args[1] == 'PATH_PREVIEW' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            request_id = request_preview()
            preview_call = gateway.send_command.await_args
            self.assertEqual(preview_call.args[:2], ('R01', 'PATH_PREVIEW'))
            sent_preview = preview_call.args[2]
            self.assertEqual({key: sent_preview[key] for key in (
                'request_id', 'x', 'y', 'yaw', 'frame_id', 'source_type', 'source_id',
                'tag_id', 'tag_revision', 'registry_revision', 'source_map_id',
                'source_map_revision', 'registration_revision', 'registration_source',
                'active_map_id', 'active_map_revision', 'canonical_map_revision')}, {
                'request_id': request_id, 'x': 2.0, 'y': 5.0, 'yaw': 0.25, 'frame_id': 'map',
                'source_type': 'TAG', 'source_id': '1', 'tag_id': 1,
                'tag_revision': tag_record['tag_revision'], 'registry_revision': registry['registry_revision'],
                'source_map_id': 'CANONICAL', 'source_map_revision': '7',
                'registration_revision': None, 'registration_source': None,
                'active_map_id': 'CANONICAL', 'active_map_revision': '7', 'canonical_map_revision': 7,
            })
            self.assertEqual(sent_preview['route_nodes'], [1])
            self.assertTrue(sent_preview['route_revision'])
            self.assertEqual(sent_preview['route_points'][-1]['tag_id'], 1)
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))
            planner_result(request_id)

            capture.send_json.reset_mock()
            send_goal(request_id, source_id='2')
            self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            send_goal(request_id)
            nav_call = gateway.send_command.await_args
            self.assertEqual(nav_call.args[:2], ('R01', 'NAVIGATE'))
            sent_goal = nav_call.args[2]
            self.assertEqual(sent_goal['route_revision'], sent_preview['route_revision'])
            self.assertEqual(sent_goal['route_nodes'], sent_preview['route_nodes'])
            self.assertEqual(sent_goal['route_points'], sent_preview['route_points'])
            self.assertEqual((sent_goal['x'], sent_goal['y'], sent_goal['yaw']), (2.0, 5.0, 0.25))
            self.assertEqual(sent_goal['preview_request_id'], request_id)

            tag = NavigationTag.objects.get(warehouse=self.warehouse_map.warehouse, tag_id=1)
            tag.x = 2.25
            tag.save(update_fields=['x', 'updated_at'])
            current_layout = dict(self.warehouse_map.layout)
            current_tags = [dict(item) for item in current_layout['navigation_tags']]
            current_tags[0]['x'] = 2.25
            self.warehouse_map.layout = {**current_layout, 'navigation_tags': current_tags}
            self.warehouse_map.save(update_fields=['layout', 'updated_at'])
            updated_registry = navigation_tag_registry(self.active_map)
            updated_tag = next(item for item in updated_registry['tags'] if item['tag_id'] == 1)
            changed_request = request_preview(tag_revision=updated_tag['tag_revision'],
                                              registry_revision=updated_registry['registry_revision'])
            tag.x = 2.5
            tag.save(update_fields=['x', 'updated_at'])
            planner_result(changed_request, goal=(2.25, 5.0, 0.25))
            self.assertEqual(runtime.path_preview_results[('R01', changed_request)]['status'], 'INVALID')
            capture.send_json.reset_mock()
            send_goal(changed_request, goal=(2.25, 5.0, 0.25))
            self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
            self.assertNotEqual(changed_request, request_id)

    def test_registered_slam_tag_preview_uses_transformed_active_map_pose(self):
        local_map = {**self.active_map, 'active_map_id': 'SLAM-session-4',
                     'active_map_revision': 'slam-rev-12', 'map_sync_status': 'LOCAL_ONLY',
                     'map_source': 'SLAM_TOOLBOX'}
        RobotMapRegistration.objects.create(
            robot_id='R01', warehouse_map=self.warehouse_map, canonical_revision=7,
            active_map_id='SLAM-session-4', active_map_revision='slam-rev-12',
            tx=10.0, ty=-2.0, yaw=math.pi / 2,
            registration_revision=1, source='survey-control-points',
        )
        registry = navigation_tag_registry(local_map, robot_id='R01')
        tag_record = next(item for item in registry['tags'] if item['tag_id'] == 1)
        capture = SimpleNamespace(send_json=AsyncMock())
        gateway = SimpleNamespace(send_command=AsyncMock(return_value={'ok': True}))
        localization_state = {'frame_id': 'map', 'map_id': 'SLAM-session-4',
                              'map_revision': 'slam-rev-12', 'map_source': 'SLAM_TOOLBOX',
                              'pose_source': 'TF'}
        request_id = 'registered-local-tag-preview'

        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'unified_navigation_blocker', return_value=None), \
                patch.object(runtime, 'robot_runtime_modes', {'R01': 'NAVIGATION'}), \
                patch.object(runtime, 'active_map_state', return_value=local_map), \
                patch.object(runtime, 'robot_bridge_online', return_value=True), \
                patch.object(runtime, 'navigation_localization_state', return_value=localization_state), \
                patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime, 'broadcast', new=AsyncMock()), \
                patch.object(runtime, 'path_preview_requests', {}), \
                patch.object(runtime, 'approved_path_previews', {}), \
                patch.object(runtime, 'path_preview_results', {}), \
                patch.object(runtime, 'expired_path_previews', {}), \
                patch.object(runtime, 'path_preview_invalidations', {}), \
                patch.object(runtime.engine, 'state', {'robots': {'R01': {
                    'active_map_pose': {'valid': True, 'frame_id': 'map',
                        'map_id': 'SLAM-session-4', 'map_revision': 'slam-rev-12',
                        'map_source': 'SLAM_TOOLBOX', 'pose_source': 'TF',
                        'x': 5.0, 'y': 0.0, 'yaw': math.pi / 2},
                }}}):
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'PATH_PREVIEW_REQUEST', 'robot_id': 'R01',
                'request_id': request_id, 'source_type': 'TAG', 'tag_id': 1,
                'tag_revision': tag_record['tag_revision'],
                'registry_revision': registry['registry_revision'],
                'frame_id': 'map', 'active_map_id': 'SLAM-session-4',
                'active_map_revision': 'slam-rev-12',
            })

        self.assertEqual(capture.send_json.await_count, 0)
        sent = gateway.send_command.await_args.args[2]
        expected = {
            'request_id': request_id, 'x': 5.0, 'y': 0.0,
            'yaw': math.pi / 2 + 0.25, 'frame_id': 'map',
            'source_type': 'TAG', 'source_id': '1', 'tag_id': 1,
            'tag_revision': tag_record['tag_revision'],
            'registry_revision': registry['registry_revision'],
            'source_map_id': 'CANONICAL', 'source_map_revision': '7',
            'registration_revision': 1, 'registration_source': 'survey-control-points',
            'map_content_revision': None,
            'active_map_id': 'SLAM-session-4', 'active_map_revision': 'slam-rev-12',
            'canonical_map_revision': 7,
        }
        self.assertEqual(gateway.send_command.await_args.args[:2], ('R01', 'PATH_PREVIEW'))
        for key, value in expected.items():
            if isinstance(value, float):
                self.assertAlmostEqual(sent[key], value)
            else:
                self.assertEqual(sent[key], value)
        self.assertEqual(sent['route_nodes'], [1])
        self.assertEqual(sent['route_points'][-1]['tag_id'], 1)
        self.assertTrue(sent['route_revision'])
