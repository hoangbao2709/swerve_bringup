import sys
import threading
import time
from array import array
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'swerve_bridge'))
from swerve_bridge.visualization_worker import VisualizationWorker
from swerve_bridge.web_map_renderer import (bounded_cloud_points, bounded_voxel_points,
    transform_points_xyz, compress_occupancy_grid, occupancy_content_signature)


def test_worker_coalesces_unlimited_wakeups_and_never_blocks_control():
    entered, release, newest = threading.Event(), threading.Event(), threading.Event()
    source = [0]
    seen = []
    def render():
        value = source[0]
        seen.append(value)
        if value == 0:
            entered.set(); assert release.wait(2)
        if value == 999:
            newest.set()
    worker = VisualizationWorker(render, lambda: .2, lambda error: (_ for _ in ()).throw(error)).start()
    try:
        assert entered.wait(1)
        start = time.monotonic()
        for value in range(1, 1000):
            source[0] = value; worker.wake()
        assert time.monotonic() - start < .1
        assert worker.pending is True and worker.wakeups_coalesced == 998
        release.set(); assert newest.wait(1)
        assert seen[:2] == [0, 999]
    finally:
        release.set(); worker.close()
    assert not worker.thread.is_alive()


def test_view_bursts_are_bounded_and_safety_manual_always_take_priority():
    import queue
    import pytest
    from swerve_bridge.control_mailbox import ControlMailbox
    from swerve_bridge.outbound_mailbox import OutboundMailbox
    inbound, outbound = ControlMailbox(), OutboundMailbox()
    for i in range(1000):
        inbound.put({'type': 'DETAIL_VIEW', 'request_id': i})
        outbound.offer({'type': 'ROBOT_DETAIL_VIEW_STATUS', 'request_id': i})
    inbound.put({'type': 'MANUAL_CMD', 'action': 'FORWARD'})
    outbound.offer({'type': 'ROBOT_CONTROL_STATUS'})
    assert inbound.get_nowait()['type'] == 'MANUAL_CMD'
    assert inbound.get_nowait()['request_id'] == 999
    with pytest.raises(queue.Empty): inbound.get_nowait()
    assert outbound.take()[1]['type'] == 'ROBOT_CONTROL_STATUS'
    assert outbound.take()[1]['request_id'] == 999
    assert len(outbound.latest) == len(outbound.critical) == 0


def test_view_ack_wakes_worker_without_recompressing_map():
    from swerve_bridge.bridge_node import SwerveBridge
    from swerve_bridge.web_map_renderer import LatestFrameBuffer
    from swerve_bridge.outbound_mailbox import OutboundMailbox
    frames, wakes = [], []
    node = SimpleNamespace(detail_view='LIDAR_3D', detail_view_epoch=4,
        last_sent_map_signature=('unchanged-map',), lidar_frame_buffer=LatestFrameBuffer(),
        outbound=OutboundMailbox(), web_cloud_epoch='bridge', robot_id='R01',
        send=frames.append, now=lambda: 'now',
        visualization_worker=SimpleNamespace(wake=lambda: wakes.append(True)))
    assert SwerveBridge.set_detail_view(node, 'GLOBAL', {'request_id': 'view-request'})
    assert node.last_sent_map_signature == ('unchanged-map',)
    assert frames[0]['state'] == 'APPLIED' and frames[0]['view_epoch'] == 5
    assert frames[0]['request_id'] == 'view-request' and wakes == [True]
    assert not SwerveBridge.set_detail_view(node, 'invalid', {})
    assert len(frames) == 1


def test_telemetry_timer_never_waits_for_tf_in_the_control_executor():
    from swerve_bridge.bridge_node import SwerveBridge
    pending = threading.Event()
    wakes = []
    def forbidden(): raise AssertionError('TF telemetry ran on the ROS timer')
    node = SimpleNamespace(telemetry_pending=pending, telemetry_timer=forbidden,
        visualization_worker=SimpleNamespace(wake=lambda: wakes.append(True)))
    SwerveBridge.request_telemetry(node)
    assert pending.is_set() and wakes == [True]


def test_native_ros_grid_encoding_is_identical_and_validates_bounds():
    import pytest
    values = [-1, 0, 50, 100]
    assert compress_occupancy_grid(array('b', values)) == compress_occupancy_grid(values)
    assert occupancy_content_signature(array('b', values)) == occupancy_content_signature(values)
    with pytest.raises(ValueError): compress_occupancy_grid(array('b', [101]))
    with pytest.raises(ValueError): compress_occupancy_grid(array('b', values), max_cells=3)


def test_identical_hold_mode_status_is_coalesced_but_safety_ack_is_not():
    from swerve_bridge.bridge_node import SwerveBridge
    frames = []
    node = SimpleNamespace(robot_id='R01', applied_mode='MANUAL', requested_mode='MANUAL',
        mode_transition_state='APPLIED', mode_request_id='mode', now=lambda: 'now', send=frames.append)
    for _ in range(1000): SwerveBridge.send_control_status(node, True, manual_refresh=True)
    assert len(frames) == 1
    SwerveBridge.send_control_status(node, True)  # STOP / mode application
    SwerveBridge.send_control_status(node, False, 'emergency stop is active')
    assert len(frames) == 3 and frames[-1]['accepted'] is False


def test_native_cloud_matches_existing_transform_filter_and_bounds():
    random = np.random.default_rng(3)
    points = random.uniform(-2, 2, (5000, 3))
    points[0] = [np.nan, 0, 0]
    raw = np.array(list(map(tuple, points)), dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8')])
    translation, quaternion = [.1, -.2, .3], [0, 0, .3, np.sqrt(.91)]
    expected = bounded_voxel_points(transform_points_xyz(points, translation, quaternion), max_points=400)
    actual = bounded_cloud_points(raw, translation, quaternion, max_points=400)
    np.testing.assert_allclose(actual, expected, atol=1e-10)
    assert len(actual) <= 400
