import asyncio
from unittest.mock import AsyncMock, patch

from django.test import SimpleTestCase

from twin.map_sync import map_sync_status
from twin.navigation_views import _map_sync_error
from twin.runtime import runtime


class MapSyncTests(SimpleTestCase):
    def test_status_transitions(self):
        self.assertEqual(map_sync_status(published_revision=1, ros_revision=None, gazebo_revision=None, ros_connected=False), 'ROS_OFFLINE')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=1, gazebo_revision=1, ros_connected=True,
                                         tag_map_revision=1, nav2_revision=1, tf_status=True), 'OUT_OF_SYNC')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True,
                                         nav2_revision=2, tag_map_revision=2, tf_status=True), 'SYNCED')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True,
                                         nav2_revision=2, tag_map_revision=None, tf_status=True), 'OUT_OF_SYNC')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True,
                                         tag_map_revision=2, tf_status=True, require_nav2=False), 'SYNCED')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True,
                                         nav2_revision=None, tag_map_revision=None, tf_status=True,
                                         require_nav2=False, require_tag_map=False), 'SYNCED')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True,
                                         nav2_revision=2, tag_map_revision=2, tf_status=False,
                                         error='map to base TF unavailable'), 'OUT_OF_SYNC')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=1, gazebo_revision=2, ros_connected=True,
                                         nav2_revision=2, tag_map_revision=2, tf_status=True,
                                         error='revision mismatch'), 'OUT_OF_SYNC')

    def test_publish_event_is_broadcast_and_sent_to_bridge(self):
        payload = {'warehouse_id': 7, 'warehouse_code': 'WH', 'map_revision': 2, 'revision': 2,
                   'published_version': 1, 'artifact_manifest': {'revision': 2}, 'artifact_dir': '/tmp/WH/2'}
        old = (runtime.published_map_revision, runtime.published_map_version, runtime.map_sync_error)
        try:
            with patch.object(runtime, 'broadcast', new=AsyncMock()) as broadcast, \
                 patch('twin.ros_bridge_consumer.registry.send', new=AsyncMock(return_value=True)) as send:
                asyncio.run(runtime.map_published(payload))
                broadcast.assert_any_call({'type': 'map.published', 'warehouse_id': 7, 'revision': 2,
                                           'published_version': 1, 'map_revision': 2,
                                           'artifact_manifest': {'revision': 2}})
                send.assert_awaited_once()
                self.assertEqual(send.await_args.args[0]['type'], 'MAP_PUBLISHED')
        finally:
            runtime.published_map_revision, runtime.published_map_version, runtime.map_sync_error = old

    def test_mission_gate_reports_revision_mismatch(self):
        old = (runtime.runtime_mode, runtime.ros_bridge_connected, runtime.published_map_revision,
               runtime.ros_map_revision, runtime.gazebo_map_revision, runtime.nav2_map_revision,
               runtime.tag_map_revision, runtime.map_tf_status, runtime.operation_mode)
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.ros_bridge_connected = True
            runtime.published_map_revision = 12
            runtime.ros_map_revision = 11
            runtime.gazebo_map_revision = 11
            runtime.nav2_map_revision = 11
            runtime.tag_map_revision = 11
            runtime.map_tf_status = True
            runtime.operation_mode = 'NAVIGATION'
            self.assertIn('map revision mismatch', _map_sync_error())
            runtime.ros_map_revision = runtime.gazebo_map_revision = 12
            runtime.nav2_map_revision = runtime.tag_map_revision = 12
            self.assertIsNone(_map_sync_error())
        finally:
            (runtime.runtime_mode, runtime.ros_bridge_connected, runtime.published_map_revision,
             runtime.ros_map_revision, runtime.gazebo_map_revision, runtime.nav2_map_revision,
             runtime.tag_map_revision, runtime.map_tf_status, runtime.operation_mode) = old

    def test_all_connected_robot_bridges_must_agree_on_revision(self):
        old = (
            runtime.connected_robot_ids.copy(), runtime.robot_map_sync.copy(),
            runtime.ros_bridge_connected, runtime.runtime_mode, runtime.operation_mode,
            runtime.published_map_revision, runtime.ros_map_revision,
            runtime.gazebo_map_revision, runtime.nav2_map_revision,
            runtime.tag_map_revision, runtime.map_tf_status, runtime.map_sync_error,
        )
        try:
            runtime.connected_robot_ids.clear()
            runtime.robot_map_sync.clear()
            runtime.connected_robot_ids.add('R01')
            runtime.ros_bridge_connected = True
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.operation_mode = 'NAVIGATION'
            runtime.published_map_revision = 12
            with patch.object(runtime, 'broadcast', new=AsyncMock()), \
                 patch.object(runtime, 'send_published_map_to_bridge', new=AsyncMock()) as resend:
                base = {
                    'ros_revision': 12, 'gazebo_revision': 12,
                    'nav2_revision': 12, 'tag_map_revision': 12, 'tf_status': True,
                }
                asyncio.run(runtime.handle_map_revision_status({**base, 'robot_id': 'R01'}))
                self.assertEqual(runtime.runtime_status_message()['map_sync_status'], 'SYNCED')
                runtime.connected_robot_ids.add('R02')
                asyncio.run(runtime.handle_map_revision_status({
                    **base, 'robot_id': 'R02', 'gazebo_revision': 11,
                }))
                status = runtime.runtime_status_message()
                self.assertEqual(status['map_sync_status'], 'OUT_OF_SYNC')
                self.assertIsNone(status['gazebo_revision'])
                self.assertIn('robot bridges disagree', status['map_sync_error'])
                self.assertEqual(set(status['robot_map_sync']), {'R01', 'R02'})
                resend.assert_not_awaited()
        finally:
            (runtime.connected_robot_ids, runtime.robot_map_sync,
             runtime.ros_bridge_connected, runtime.runtime_mode, runtime.operation_mode,
             runtime.published_map_revision, runtime.ros_map_revision,
             runtime.gazebo_map_revision, runtime.nav2_map_revision,
             runtime.tag_map_revision, runtime.map_tf_status, runtime.map_sync_error) = old
