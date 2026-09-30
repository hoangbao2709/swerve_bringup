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
