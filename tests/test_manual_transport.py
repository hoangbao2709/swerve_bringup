import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'swerve_bridge'))
from manual_refresh import ManualRefreshWorker
from swerve_bridge.outbound_mailbox import OutboundMailbox


@pytest.mark.parametrize('method,payload', [
    ('navigate', {'target_tag_id': 1101}),
    ('navigate_pose', {'x': 1., 'y': 2., 'yaw': 0., 'frame_id': 'map'}),
])
def test_offline_nav_server_never_blocks_shared_control_executor(method, payload):
    from swerve_bridge.bridge_node import SwerveBridge
    errors = []
    def forbidden_wait(**kwargs):
        raise AssertionError('blocking discovery in ROS control executor')
    client = SimpleNamespace(server_is_ready=lambda: False, wait_for_server=forbidden_wait)
    node = SimpleNamespace(robot_id='R01', local_map_load_pending=False,
        loaded_local_map_id=None, emergency_stop_active=False, control_mode='AUTONOMOUS',
        active_goal=None, active_pose_goal=None, goal_request_pending=False,
        nav_client=client, nav_pose_client=client,
        send_nav_status=lambda context, status, reason: errors.append((status, reason)))
    getattr(SwerveBridge, method)(node, payload)
    assert errors == [('FAILED', ('GoToTag' if method == 'navigate' else 'NavigateToPose')
                       + ' action server unavailable')]


def test_refresh_is_independent_of_slow_probe_and_stop_is_final():
    sent = []
    sender = ManualRefreshWorker(sent.append, 'R01', .01).start()
    sender.hold('FORWARD')
    time.sleep(.12)  # main diagnostic thread is intentionally unavailable
    sender.stop()
    count = len(sent)
    time.sleep(.05)
    assert len(sent) == count and sent[-1]['action'] == 'STOP'
    sender.close()
    assert len([row for row in sent if row['action'] == 'FORWARD']) >= 5
    assert [r['sequence_id'] for r in sent] == list(range(1, len(sent) + 1))
    assert not sender.thread.is_alive()


def test_refresh_period_does_not_add_network_duration_or_queue_old_ticks():
    starts = []
    def send(message):
        if message['action'] != 'STOP':
            starts.append(time.monotonic())
            time.sleep(.03)
    sender = ManualRefreshWorker(send, 'R01', .06).start()
    try:
        sender.hold('FORWARD')
        time.sleep(.28)
        sender.stop()
    finally:
        sender.close()
    assert len(starts) >= 4
    assert max(b - a for a, b in zip(starts, starts[1:])) < .085


def test_stop_invalidates_blocked_old_send_before_wire_stop():
    entered, release = threading.Event(), threading.Event()
    sent = []
    def send(message):
        if message['action'] == 'FORWARD':
            entered.set()
            assert release.wait(2)
        sent.append(message)
    sender = ManualRefreshWorker(send, 'R01', .01).start()
    sender.hold('FORWARD')
    assert entered.wait(1)
    stopper = threading.Thread(target=sender.stop)
    stopper.start()
    time.sleep(.02)
    release.set()
    stopper.join(1)
    sender.close()
    assert [r['action'] for r in sent] == ['FORWARD', 'STOP', 'STOP']


def test_outbound_is_latest_only_and_critical_is_fifo_priority():
    out = OutboundMailbox(capacity=2)
    for index in range(1000):
        out.offer({'type': 'LIDAR_MAP_3D', 'index': index})
    out.offer({'type': 'ROBOT_CONTROL_STATUS', 'index': 1})
    out.offer({'type': 'ROBOT_CONTROL_STATUS', 'index': 2})
    with pytest.raises(BufferError):
        out.offer({'type': 'ROBOT_CONTROL_STATUS', 'index': 3})
    assert [out.take()[1]['index'] for _ in range(3)] == [1, 2, 999]
    assert out.dropped == 999
    out.offer({'type': 'LIDAR_MAP_3D'})
    epoch, _ = out.take()
    out.clear()
    assert not out.current(epoch) and out.take(.001) is None


def test_heartbeat_remains_available_while_network_ping_is_blocked():
    from swerve_bridge.bridge_node import SwerveBridge
    entered, release = threading.Event(), threading.Event()
    stop = threading.Event()
    mailbox = OutboundMailbox()
    wire = []

    def ping():
        entered.set()
        assert release.wait(2)

    socket = SimpleNamespace(ping=ping, send=wire.append)
    def forbidden_diagnostics():
        raise AssertionError('heavy graph diagnostics ran inside the ROS heartbeat callback')
    node = SimpleNamespace(stop_event=stop, outbound=mailbox, ws=socket,
        ws_lock=threading.Lock(), control_timing={},
        send=lambda payload: mailbox.offer(payload),
        robot_id='R01', nav_state='READY', runtime_state='NAVIGATION',
        applied_mode='MANUAL', slam_paused=False, controller_states=[], gazebo_rtf=.2,
        now=lambda: 'timestamp', _mapping_elapsed_s=lambda: 0,
        collect_diagnostics=forbidden_diagnostics, detail_errors=lambda diagnostics: [],
        send_map_revision_status=lambda: None)
    worker = threading.Thread(target=SwerveBridge.outbound_sender, args=(node,))
    worker.start()
    try:
        mailbox.offer({'type': '_SOCKET_PING'})
        assert entered.wait(1)
        callback_done = threading.Event()
        callback = threading.Thread(target=lambda: (
            SwerveBridge.heartbeat_timer(node), callback_done.set()))
        callback.start()
        assert callback_done.wait(.5), 'ROS heartbeat waited for network ping'
        callback.join(1)
        for _ in range(100):
            mailbox.offer({'type': '_SOCKET_PING'})
        assert '_SOCKET_PING' in mailbox.latest
        assert len(mailbox.critical) == 0  # diagnostics are off the ROS callback
        assert all('_SOCKET_PING' not in item for item in wire)
    finally:
        stop.set()
        release.set()
        mailbox.clear()
        worker.join(2)
    assert not worker.is_alive()


def test_map_validation_cache_invalidates_grid_yaml_and_image(tmp_path):
    from swerve_bridge.bridge_node import SwerveBridge
    yaml = tmp_path / 'map.yaml'
    image = tmp_path / 'map.pgm'
    yaml.write_text('image: map.pgm\n')
    image.write_bytes(b'first')
    calls = []
    node = SimpleNamespace(latest_map=object(), latest_map_signature=('grid', 1),
        nav2_map_file=yaml, nav2_configured_revision=21,
        _verify_loaded_nav2_revision=lambda present: calls.append(present) or 21)
    def check():
        return SwerveBridge._loaded_nav2_revision(node, True)
    assert check() == check() == 21 and len(calls) == 1
    node.latest_map_signature = ('grid', 2)
    assert check() == 21 and len(calls) == 2
    image.write_bytes(b'changed image')
    assert check() == 21 and len(calls) == 3
    yaml.write_text('image: map.pgm\n# changed\n')
    assert check() == 21 and len(calls) == 4
    assert SwerveBridge._loaded_nav2_revision(node, False) is None
