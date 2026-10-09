import os
from pathlib import Path
import json
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from waretwin_soak_test import process_tree_metrics, read_runtime_status, system_metrics


class SoakMetricsTests(unittest.TestCase):
    def test_process_tree_reads_owned_process_cpu_and_rss(self):
        cpu_seconds, rss_bytes = process_tree_metrics([os.getpid()])
        self.assertGreaterEqual(cpu_seconds, 0.0)
        self.assertGreater(rss_bytes, 0)

    def test_system_metrics_reports_host_memory_when_procfs_is_available(self):
        cpu_ticks, available_bytes = system_metrics()
        self.assertIsNotNone(cpu_ticks)
        self.assertGreater(available_bytes, 0)

    def test_runtime_status_reads_authoritative_state_and_owned_children(self):
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            state = root / 'state' / 'waretwin'
            runtime = root / 'runtime'
            state.mkdir(parents=True)
            runtime.mkdir()
            (state / 'active-runtime.json').write_text(json.dumps({'runtime_dir': str(runtime)}))
            (runtime / 'web-status.json').write_text(json.dumps({
                'state': 'BRIDGE_DISCONNECTED', 'pid': os.getpid(),
                'children': [{'pid': os.getpid(), 'running': True},
                             {'pid': os.getpid() + 100000, 'running': False}],
            }))
            self.assertEqual(read_runtime_status(root / 'state'),
                             ('BRIDGE_DISCONNECTED', [os.getpid(), os.getpid()]))


if __name__ == '__main__':
    unittest.main()
