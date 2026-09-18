import asyncio
from unittest.mock import AsyncMock, patch

from django.test import SimpleTestCase

from twin.map_sync import map_sync_status
from twin.navigation_views import _map_sync_error
from twin.runtime import runtime


class MapSyncTests(SimpleTestCase):
    def test_status_transitions(self):
        self.assertEqual(map_sync_status(published_revision=1, ros_revision=None, gazebo_revision=None, ros_connected=False), 'ROS_OFFLINE')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=1, gazebo_revision=1, ros_connected=True), 'OUT_OF_SYNC')
        self.assertEqual(map_sync_status(published_revision=2, ros_revision=2, gazebo_revision=2, ros_connected=True), 'SYNCED')

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
               runtime.ros_map_revision, runtime.gazebo_map_revision)
        try:
            runtime.runtime_mode = 'GAZEBO_ROS'
            runtime.ros_bridge_connected = True
            runtime.published_map_revision = 12
            runtime.ros_map_revision = 11
            runtime.gazebo_map_revision = 11
            self.assertIn('map revision mismatch', _map_sync_error())
            runtime.ros_map_revision = runtime.gazebo_map_revision = 12
            self.assertIsNone(_map_sync_error())
        finally:
            (runtime.runtime_mode, runtime.ros_bridge_connected, runtime.published_map_revision,
             runtime.ros_map_revision, runtime.gazebo_map_revision) = old
