from asgiref.sync import async_to_sync
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from django.test import TestCase
from django.contrib.auth.models import User
from django.test import Client
from unittest.mock import patch

from accounts.models import ApiToken
from twin.models import NavigationTag, NavigationTagEdge, WarehouseMap
from twin.navigation_targets import NavigationTargetError, navigation_tag_registry, resolve_navigation_target
from twin.runtime import runtime
from twin.tag_navigation import shortest_tag_route
from twin.warehouse_services import sync_from_layout


class NavigationGraphSyncTests(TestCase):
    def layout(self):
        return {
            'id': 'graph-test', 'name': 'Graph Test', 'units': 'm',
            'size': {'width': 20, 'depth': 10, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [20, 0], [20, 10], [0, 10]]}],
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


class NavigationTargetResolutionTests(TestCase):
    def setUp(self):
        self.old_robot_mode = runtime.robot_runtime_modes.get('R01')
        self.old_operation_mode = runtime.operation_mode
        runtime.robot_runtime_modes.pop('R01', None)
        runtime.operation_mode = 'NAVIGATION'
        self.layout = {
            'id': 'target-resolution-test', 'name': 'Target Resolution Test', 'units': 'm',
            'size': {'width': 20, 'depth': 10, 'height': 4},
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [20, 0], [20, 10], [0, 10]]}],
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

    def tearDown(self):
        runtime.operation_mode = self.old_operation_mode
        if self.old_robot_mode is None:
            runtime.robot_runtime_modes.pop('R01', None)
        else:
            runtime.robot_runtime_modes['R01'] = self.old_robot_mode
        super().tearDown()

    def resolve(self, tag_id, active_map=None):
        return resolve_navigation_target(
            robot_id='R01', source_type='TAG', active_map=active_map or self.active_map,
            tag_id=tag_id,
        )

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
        self.assertEqual((point['source_type'], point['source_id']), ('MAP_POINT', None))

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

    def test_robot_tag_api_is_authenticated_and_returns_only_map_compatible_registry(self):
        user = User.objects.create_user(username='tag-registry-test', password='test-only-password')
        token = ApiToken.issue(user)
        client = Client(HTTP_AUTHORIZATION=f'Bearer {token.key}')
        with patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'):
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
        self.assertFalse(response.json()['compatible'])
        self.assertIn('SLAM map', response.json()['reason'])

        anonymous = Client().get('/api/robots/R01/navigation-tags')
        self.assertEqual(anonymous.status_code, 401)

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
            }, None)
            return request_id

        def planner_result(request_id, goal=(2.0, 5.0, 0.25)):
            async_to_sync(runtime.handle_ros_message)({
                'type': 'PATH_PREVIEW_RESULT', 'robot_id': 'R01', 'request_id': request_id,
                'status': 'VALID', 'path': [[0.0, 0.0], [goal[0], goal[1]]],
                'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]},
                'active_map_id': 'CANONICAL', 'active_map_revision': '7',
            })

        def send_goal(request_id, source_id='1', source_type='TAG', goal=(2.0, 5.0, 0.25)):
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': goal[0], 'y': goal[1], 'yaw': goal[2],
                'frame_id': 'map', 'preview_request_id': request_id,
                'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                'source_type': source_type, 'source_id': source_id,
            }, None)

        with patch.object(runtime, 'runtime_mode', 'GAZEBO_ROS'), \
                patch.object(runtime, 'operation_mode', 'NAVIGATION'), \
                patch.object(runtime, 'active_map_state', return_value=self.active_map), \
                patch.object(runtime, 'robot_bridge_online', return_value=True), \
                patch.object(runtime, 'navigation_localization_state', return_value=localization_state), \
                patch.object(runtime, 'gateway', return_value=gateway), \
                patch.object(runtime, 'broadcast', new=AsyncMock()), \
                patch.object(runtime, 'path_preview_requests', {}), \
                patch.object(runtime, 'approved_path_previews', {}), \
                patch.object(runtime, 'path_preview_results', {}), \
                patch.object(runtime, 'expired_path_previews', {}), \
                patch.object(runtime, 'path_preview_invalidations', {}):
            async_to_sync(runtime.handle_message)(capture, {
                'type': 'NAV_GOAL', 'robot_id': 'R01', 'x': 2.0, 'y': 5.0, 'yaw': 0.25,
                'frame_id': 'map', 'active_map_id': 'CANONICAL', 'active_map_revision': '7',
                'source_type': 'TAG', 'source_id': '1',
            }, None)
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
            gateway.send_command.assert_awaited_with('R01', 'PATH_PREVIEW', {
                'request_id': request_id, 'x': 2.0, 'y': 5.0, 'yaw': 0.25, 'frame_id': 'map',
                'source_type': 'TAG', 'source_id': '1', 'tag_id': 1,
                'tag_revision': tag_record['tag_revision'], 'registry_revision': registry['registry_revision'],
                'active_map_id': 'CANONICAL', 'active_map_revision': '7', 'canonical_map_revision': 7,
            })
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))
            planner_result(request_id)

            capture.send_json.reset_mock()
            send_goal(request_id, source_id='2')
            self.assertEqual(capture.send_json.await_args.args[0]['code'], 'PATH_PREVIEW_INVALID')
            self.assertFalse(any(call.args[1] == 'NAVIGATE' for call in gateway.send_command.await_args_list))

            capture.send_json.reset_mock()
            send_goal(request_id)
            gateway.send_command.assert_awaited_with('R01', 'NAVIGATE', {
                'x': 2.0, 'y': 5.0, 'yaw': 0.25, 'frame_id': 'map',
                'source_type': 'TAG', 'source_id': '1', 'tag_id': 1,
                'tag_revision': tag_record['tag_revision'], 'registry_revision': registry['registry_revision'],
                'preview_request_id': request_id, 'active_map_id': 'CANONICAL',
                'active_map_revision': '7', 'canonical_map_revision': 7,
            })

            tag = NavigationTag.objects.get(warehouse=self.warehouse_map.warehouse, tag_id=1)
            tag.x = 2.25
            tag.save(update_fields=['x', 'updated_at'])
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
