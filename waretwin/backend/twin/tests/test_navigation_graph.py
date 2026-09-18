from django.test import TestCase

from twin.models import NavigationTag, NavigationTagEdge
from twin.tag_navigation import shortest_tag_route
from twin.warehouse_services import sync_from_layout


class NavigationGraphSyncTests(TestCase):
    def layout(self):
        return {
            'id': 'graph-test', 'name': 'Graph Test', 'units': 'm',
            'size': {'width': 20, 'depth': 10, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [20, 0], [20, 10], [0, 10]]}],
            'navigation_tags': [
                {'uuid': 't1', 'tag_id': 1, 'floor_id': 'F1', 'x': 2, 'y': 5, 'yaw': 0},
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
        self.assertEqual(shortest_tag_route(1, 3), [1, 2, 3])
        with self.assertRaises(ValueError):
            shortest_tag_route(3, 1)

    def test_bidirectional_and_stale_edges(self):
        layout = self.layout()
        layout['navigation_edges'][0]['direction'] = 'bidirectional'
        layout['navigation_edges'][0]['bidirectional'] = True
        sync_from_layout(layout, prune=True)
        self.assertEqual(shortest_tag_route(2, 1), [2, 1])
        layout['navigation_edges'] = layout['navigation_edges'][:1]
        sync_from_layout(layout, prune=True)
        self.assertEqual(NavigationTagEdge.objects.count(), 1)
