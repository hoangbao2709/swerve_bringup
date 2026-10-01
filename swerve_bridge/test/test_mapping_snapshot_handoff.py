from types import SimpleNamespace
from collections import deque
from unittest.mock import Mock

import pytest

from swerve_bridge import bridge_node
from swerve_bridge.bridge_node import SwerveBridge


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
