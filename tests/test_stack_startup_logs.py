"""Startup must never classify a previous run's errors as a new failure."""
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_prepare_log_removes_historical_error_before_delayed_writer_and_preserves_evidence(tmp_path):
    old = "[ERROR] previous controller activation failed\n[gzserver-1]: process started with pid [123]\n"
    (tmp_path / 'ros.log').write_text(old)
    result = subprocess.run(
        ['bash', '-c', '''
source "$1/scripts/stack_common.sh"
STACK_LOG_DIR="$2"
stack_prepare_log ros
# The new background writer has not started yet. A probe must see no old
# error or process marker even during this interval.
test -f "$STACK_LOG_DIR/ros.log" && test ! -s "$STACK_LOG_DIR/ros.log"
''', '_', str(ROOT), str(tmp_path)], capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    archives = list(tmp_path.glob('ros.log.previous.*'))
    assert len(archives) == 1
    assert archives[0].read_text() == old
    assert (tmp_path / 'ros.log').read_text() == ''


def test_log_initialization_precedes_async_ros_process_and_startup_error_probe():
    source = (ROOT / 'scripts/start_stack.sh').read_text()
    initial = source.index('stack_prepare_log ros\n')
    bridge = source.index('stack_prepare_log ros_bridge\n')
    background = source.index("setsid bash -c '\n", initial)
    error_probe = source.index("if rg -q '\\[ERROR\\]", background)
    assert initial < bridge < background < error_probe
