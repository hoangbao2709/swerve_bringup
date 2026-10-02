from types import SimpleNamespace
from collections import deque
from unittest.mock import Mock

import pytest

from swerve_bridge import bridge_node
from swerve_bridge.bridge_node import SwerveBridge
from swerve_bridge.web_map_renderer import BoundedVoxelMap, LatestFrameBuffer


def test_map_callback_only_replaces_latest_reference_and_wakes_worker(monkeypatch):
    bridge = object.__new__(SwerveBridge)
    bridge.latest_map = None
    bridge.latest_map_received_monotonic = None
    bridge.latest_map_generation = 0
    bridge.map_callback_count = 0
    bridge.last_map_monotonic = None
    bridge.map_intervals = deque(maxlen=30)
    bridge.map_snapshot_worker = SimpleNamespace(wake=Mock())
    monkeypatch.setattr(
        bridge_node, 'occupancy_content_signature',
        lambda _cells: (_ for _ in ()).throw(AssertionError('hash must run on worker')),
    )

    first, newest = object(), object()
    bridge.map_cb(first)
    bridge.map_cb(newest)

    assert bridge.latest_map is newest
    assert bridge.latest_map_generation == 2
    assert bridge.map_callback_count == 2
    assert bridge.latest_map_received_monotonic is not None
    assert bridge.map_snapshot_worker.wake.call_count == 2


def test_map_version_changes_only_when_geometry_or_content_signature_changes(monkeypatch):
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'MAPPING'
    bridge.mapping_session_id = 'session-42'
    bridge.mapping_map_revision = None
    bridge.mapping_map_version = 0
    bridge.ros_map_revision = 21
    bridge.latest_map_generation = 1
    bridge.processed_map_generation = 0
    bridge.latest_map_signature = None
    bridge.latest_map = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(width=2, height=1, resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[-1, 100],
    )
    bridge._confirm_local_map_if_ready = lambda: None
    signatures = iter(('same', 'same', 'changed'))
    monkeypatch.setattr(bridge_node, 'occupancy_content_signature', lambda _data: next(signatures))

    bridge._update_latest_map_signature()
    assert bridge.mapping_map_version == 1
    bridge.latest_map_generation = 2
    bridge._update_latest_map_signature()
    assert bridge.mapping_map_version == 1
    bridge.latest_map_generation = 3
    bridge._update_latest_map_signature()
    assert bridge.mapping_map_version == 2


def test_map_payload_is_compressed_once_per_version_and_cached_for_reconnect(monkeypatch):
    bridge = object.__new__(SwerveBridge)
    bridge._update_latest_map_signature = lambda: None
    bridge.latest_map_signature = 'signature-1'
    bridge.processed_map_payload_signature = None
    bridge.processed_map_generation = bridge.latest_map_generation = 1
    bridge.processed_map = SimpleNamespace(
        header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=3, nanosec=0)),
        info=SimpleNamespace(width=2, height=1, resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[-1, 100],
    )
    bridge.runtime_state = 'MAPPING'
    bridge.mapping_session_id = 'session-42'
    bridge.mapping_map_version = 1
    bridge.mapping_map_revision = 'revision'
    bridge.ros_map_revision = 21
    bridge.robot_id = 'R01'
    bridge.active_map_identity = lambda: {'active_map_id': 'SLAM-session-42',
        'active_map_revision': 'revision', 'canonical_map_revision': '21'}
    bridge.visualization_metrics = {'map_compressions': 0}
    bridge.latest_map_statistics = None
    bridge.last_sent_map_signature = None
    bridge.processed_map_payload = None
    bridge.send = Mock(return_value=True)
    compressed = Mock(return_value='encoded')
    monkeypatch.setattr(bridge_node, 'compress_occupancy_grid', compressed)
    monkeypatch.setattr(bridge_node, 'occupancy_grid_statistics', lambda _cells: {
        'known_cells': 1, 'occupied_cells': 1, 'free_cells': 0, 'ambiguous_cells': 0,
    })

    bridge.map_snapshot_timer()
    assert bridge.send.call_count == 1
    assert bridge.send.call_args.args[0]['map']['map_version'] == 1
    assert bridge.send.call_args.args[0]['map']['unknown_cells'] == 1
    assert bridge.send.call_args.args[0]['map']['explored_area_m2'] == pytest.approx(0.0025)
    bridge.map_snapshot_timer()
    assert compressed.call_count == 1
    bridge.last_sent_map_signature = None
    bridge.send.reset_mock(return_value=True)
    bridge.map_snapshot_timer()
    assert bridge.send.call_count == 1
    assert compressed.call_count == 1


def test_map_identity_change_rebuilds_snapshot_without_an_occupancy_change(monkeypatch):
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.ros_map_revision = 21
    bridge.loaded_local_map_id = None
    bridge.loaded_local_map_revision = None
    bridge.mapping_map_version = 0
    bridge.latest_map_generation = 1
    bridge.processed_map_generation = 0
    bridge.latest_map_signature = None
    bridge.processed_map_payload_signature = None
    bridge.latest_map = SimpleNamespace(
        header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=3, nanosec=0)),
        info=SimpleNamespace(width=2, height=1, resolution=0.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[-1, 100],
    )
    bridge._confirm_local_map_if_ready = lambda: None
    bridge.visualization_metrics = {'map_compressions': 0}
    bridge.latest_map_statistics = None
    bridge.processed_map = None
    bridge.processed_map_payload = None
    bridge.last_sent_map_signature = None
    bridge.robot_id = 'R01'
    bridge.send = Mock(return_value=True)
    compressed = Mock(return_value='encoded')
    content_signature = Mock(return_value='stable-raster-hash')
    monkeypatch.setattr(bridge_node, 'compress_occupancy_grid', compressed)
    monkeypatch.setattr(bridge_node, 'occupancy_content_signature', content_signature)
    monkeypatch.setattr(bridge_node, 'occupancy_grid_statistics', lambda _cells: {
        'known_cells': 1, 'occupied_cells': 1, 'free_cells': 0, 'ambiguous_cells': 0,
    })

    bridge.map_snapshot_timer()
    canonical_payload = bridge.send.call_args.args[0]['map']
    assert canonical_payload['active_map_id'] == 'CANONICAL'
    assert canonical_payload['map_source'] == 'NAV2_MAP'

    bridge.loaded_local_map_id = 'saved-local-map'
    bridge.loaded_local_map_revision = 'local-revision'
    bridge.last_sent_map_signature = None
    bridge.map_snapshot_timer()

    local_payload = bridge.send.call_args.args[0]['map']
    assert local_payload['active_map_id'] == 'saved-local-map'
    assert local_payload['active_map_revision'] == 'local-revision'
    assert local_payload['map_source'] == 'LOCAL_MAP'
    assert content_signature.call_count == 1
    assert compressed.call_count == 2


def test_confirming_local_map_wakes_snapshot_worker():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.ros_map_revision = 21
    bridge.loaded_local_map_id = None
    bridge.loaded_local_map_revision = None
    bridge.pending_local_map_load = {
        'service_confirmed': True, 'baseline_map_count': 0,
        'yaml_path': '/unused/map.yaml', 'map_id': 'saved-local-map',
        'map_revision': 'local-revision', 'data': {},
    }
    bridge.map_callback_count = 1
    bridge.latest_map = object()
    bridge.local_map_load_pending = True
    bridge.last_sent_map_signature = 'canonical-signature'
    bridge.robot_id = 'R01'
    bridge.map_snapshot_worker = SimpleNamespace(wake=Mock())
    bridge._local_map_matches_yaml = Mock(return_value=True)
    bridge.now = Mock(return_value=3.0)
    bridge.send = Mock(return_value=True)
    bridge._send_local_control_result = Mock()

    bridge._confirm_local_map_if_ready()

    assert bridge.loaded_local_map_id == 'saved-local-map'
    assert bridge.loaded_local_map_revision == 'local-revision'
    assert bridge.last_sent_map_signature is None
    bridge.map_snapshot_worker.wake.assert_called_once_with()


def test_requesting_global_view_replays_cached_map_for_late_client():
    bridge = object.__new__(SwerveBridge)
    bridge.detail_view = 'LIDAR_2D'
    bridge.detail_view_epoch = 2
    bridge.lidar_frame_buffer = SimpleNamespace(clear=Mock())
    bridge.outbound = SimpleNamespace(discard_views=Mock())
    bridge.last_web_cloud_source_stamp = 'old-cloud'
    bridge.last_scan_publish_monotonic = 4.0
    bridge.last_cloud_publish_monotonic = 5.0
    bridge.view_timing = {}
    bridge.web_cloud_epoch = 'bridge-epoch'
    bridge.robot_id = 'R01'
    bridge.now = Mock(return_value=10.0)
    bridge.send = Mock(return_value=True)
    bridge.visualization_worker = SimpleNamespace(wake=Mock())
    bridge.map_snapshot_worker = SimpleNamespace(wake=Mock())
    bridge.last_sent_map_signature = 'already-sent-to-another-client'
    bridge.accumulated_slam_cloud = BoundedVoxelMap(max_points=5, voxel_size=0.1)
    bridge.accumulated_slam_cloud.update([(1.0, 2.0, 0.3)])

    assert bridge.set_detail_view('GLOBAL', {'request_id': 'new-client-view'}) is True

    assert bridge.last_sent_map_signature is None
    bridge.map_snapshot_worker.wake.assert_called_once_with()
    bridge.visualization_worker.wake.assert_called_once_with()
    assert bridge.send.call_args.args[0]['type'] == 'ROBOT_DETAIL_VIEW_STATUS'
    assert bridge.accumulated_slam_cloud.snapshot() == [(1.0, 2.0, 0.3)]


def test_mapping_map_identity_is_session_scoped_and_never_canonical():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'MAPPING'
    bridge.mapping_session_id = 'session-42'
    bridge.mapping_map_revision = 'cells-abc'
    bridge.ros_map_revision = 17
    bridge.loaded_local_map_id = 'old-loaded-map'
    bridge.loaded_local_map_revision = 'old-revision'

    identity = bridge.active_map_identity()

    assert identity == {
        'active_map_id': 'SLAM-session-42',
        'active_map_revision': 'cells-abc',
        'canonical_map_revision': '17',
    }


def test_3d_cloud_accumulates_in_slam_map_while_global_is_visible(monkeypatch):
    import numpy as np
    import time

    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'MAPPING'
    bridge.detail_view = 'GLOBAL'
    bridge.detail_view_epoch = 2
    bridge.view_timing = {'request_id': 'global-view'}
    bridge.latest_filtered_cloud_monotonic = time.monotonic()
    bridge.mapping_session_id = 'session-42'
    bridge.accumulated_slam_cloud = BoundedVoxelMap(max_points=20, voxel_size=0.04)
    bridge.accumulated_slam_cloud_session = 'session-42'
    bridge.slam_trajectory = deque([(0.0, 0.0), (1.0, 0.0)])
    bridge.latest_slam_pose = {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'frame_id': 'map',
        'map_id': 'SLAM-session-42', 'map_source': 'SLAM_TOOLBOX',
        'pose_source': 'TF', 'mapping_session_id': 'session-42', 'valid': True}
    bridge.web_cloud_revision = 0
    bridge.web_cloud_epoch = 'bridge-1'
    bridge.last_web_cloud_source_stamp = None
    bridge.lidar_frame_buffer = LatestFrameBuffer()
    bridge.lidar_output_intervals = deque(maxlen=30)
    bridge.filtered_cloud_intervals = deque(maxlen=30)
    bridge.raw_cloud_intervals = deque(maxlen=30)
    bridge.detail_callback_intervals = deque(maxlen=30)
    bridge.visualization_metrics = {}
    bridge.robot_id = 'R01'
    bridge._local_path = lambda: []
    bridge._local_goal = lambda: None
    bridge._frequency = lambda _values: 0.0
    bridge.send = Mock(return_value=True)
    bridge.get_parameter = lambda name: SimpleNamespace(value={
        'map_frame': 'map', 'lidar_max_3d_points': 10,
        'mapping_sensor_tf_timeout_s': 2.5,
        'lidar_3d_min_range': 0.1, 'lidar_3d_max_range': 10.0,
        'lidar_3d_min_height': -1.0, 'lidar_3d_max_height': 3.0,
        'lidar_3d_voxel_size': 0.04,
    }[name])
    bridge.tf_buffer = SimpleNamespace(lookup_transform=Mock())
    bridge.tf_buffer.lookup_transform.return_value.transform = SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    monkeypatch.setattr(bridge_node.point_cloud2, 'read_points',
        lambda _source, **_kwargs: np.array([(1.0, 0.0, 0.2)],
            dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]))
    monkeypatch.setattr(bridge_node.Time, 'from_msg', staticmethod(lambda stamp: stamp))

    def cloud(stamp):
        return SimpleNamespace(header=SimpleNamespace(frame_id='lidar_link',
            stamp=SimpleNamespace(sec=stamp, nanosec=0)))

    bridge.latest_filtered_cloud = cloud(1)
    SwerveBridge._render_lidar_3d(bridge)
    lookup = bridge.tf_buffer.lookup_transform.call_args
    assert lookup.args[:3] == ('map', 'lidar_link', SimpleNamespace(sec=1, nanosec=0))
    assert lookup.kwargs['timeout'].nanoseconds == 2_500_000_000
    assert len(bridge.accumulated_slam_cloud) == 1
    assert not bridge.lidar_frame_buffer.pending

    bridge.latest_filtered_cloud = cloud(2)
    bridge.tf_buffer.lookup_transform.return_value.transform.translation.x = 1.0
    SwerveBridge._render_lidar_3d(bridge)
    assert len(bridge.accumulated_slam_cloud) == 2

    bridge.detail_view = 'LIDAR_3D'
    bridge.detail_view_epoch += 1
    bridge.last_web_cloud_source_stamp = None
    SwerveBridge._render_lidar_3d(bridge)
    frame = bridge.lidar_frame_buffer.take(timeout=0.01)
    assert frame['frame_id'] == 'map'
    assert frame['accumulated'] is True
    assert frame['accumulation_mode'] == 'SLAM_VISUALIZATION_VOXEL_MAP'
    assert frame['point_count'] == 2
    assert frame['slam_pose']['mapping_session_id'] == 'session-42'
    assert len(frame['trajectory']) == 2
