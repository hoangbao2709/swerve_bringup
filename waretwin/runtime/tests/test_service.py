import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from waretwin.runtime.service import Runtime, stop_group


class RuntimeSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='waretwin-service-test-')
        self.root = Path(self.temp.name)
        self.share = self.root / 'share'
        self.share.mkdir(mode=0o700)
        (self.share / 'frontend').mkdir()
        (self.share / 'frontend' / 'index.html').write_text('<html></html>')

    def tearDown(self):
        self.temp.cleanup()

    def args(self, runtime_dir, **values):
        fields = dict(share=str(self.share), runtime_dir=str(runtime_dir), config_file='',
                      backend_python='', backend_host='', backend_port='', frontend_host='',
                      frontend_port='', ros_domain_id='', full_stack=False, use_sim='true')
        fields.update(values)
        return SimpleNamespace(**fields)

    def create_runtime(self, name, extra_env=None, **args):
        runtime_dir = self.root / name
        runtime_dir.mkdir(mode=0o700)
        state_dir = self.root / f'{name}-state'
        state_dir.mkdir(mode=0o700)
        environment = {'XDG_STATE_HOME': str(state_dir), 'BACKEND_HOST': '127.0.0.1',
                       'BACKEND_PORT': '18000', 'FRONTEND_HOST': '127.0.0.1',
                       'FRONTEND_PORT': '18001', 'WARETWIN_RUNTIME_MODE': 'LOCAL_SIM',
                       'WARETWIN_OPERATOR_AUTH_MODE': 'LOCAL_LOOPBACK',
                       'WARETWIN_OPERATOR_ALLOWED_ORIGINS': 'http://127.0.0.1:18001',
                       'CORS_ALLOWED_ORIGINS': 'http://127.0.0.1:18001'}
        environment.update(extra_env or {})
        with patch.dict(os.environ, environment, clear=False):
            return Runtime(self.args(runtime_dir, **args))

    def assert_prepare_rejected(self, runtime, state_name, error, message):
        previous_umask = os.umask(0o077)
        try:
            with patch.dict(os.environ, {'XDG_STATE_HOME': str(self.root / state_name)}):
                with self.assertRaisesRegex(error, message):
                    runtime.prepare()
        finally:
            os.umask(previous_umask)

    def test_local_loopback_fails_closed_on_wildcard_bind(self):
        runtime = self.create_runtime('wildcard', {'BACKEND_HOST': '0.0.0.0'})
        try:
            self.assert_prepare_rejected(runtime, 'wildcard-state', ValueError,
                                         'LOCAL_LOOPBACK requires backend_host')
        finally:
            if getattr(runtime, 'owns_lock', False):
                runtime.lock.close()

    def test_protected_lan_fails_before_start_without_gateway_credentials(self):
        runtime = self.create_runtime('protected', {
            'WARETWIN_OPERATOR_AUTH_MODE': 'PROTECTED_LAN',
            'WARETWIN_OPERATOR_GATEWAY_SECRET': '',
        })
        try:
            self.assert_prepare_rejected(runtime, 'protected-state', ValueError,
                                         'requires WARETWIN_OPERATOR_GATEWAY_SECRET')
        finally:
            if getattr(runtime, 'owns_lock', False):
                runtime.lock.close()

    def test_protected_lan_rejects_remote_or_wildcard_proxy_networks(self):
        runtime = self.create_runtime('remote-proxy', {
            'WARETWIN_OPERATOR_AUTH_MODE': 'PROTECTED_LAN',
            'WARETWIN_OPERATOR_GATEWAY_SECRET': 'x' * 48,
            'WARETWIN_PUBLIC_ORIGINS': 'https://robot-hmi.example.test',
            'WARETWIN_TRUSTED_PROXY_CIDRS': '0.0.0.0/0',
        })
        try:
            self.assert_prepare_rejected(runtime, 'remote-proxy-state', ValueError,
                                         'trusted proxy CIDRs must be loopback-only')
        finally:
            if getattr(runtime, 'owns_lock', False):
                runtime.lock.close()

    def test_protected_lan_services_must_stay_behind_loopback_gateway(self):
        runtime = self.create_runtime('remote-bind', {
            'WARETWIN_OPERATOR_AUTH_MODE': 'PROTECTED_LAN',
            'WARETWIN_OPERATOR_GATEWAY_SECRET': 'x' * 48,
            'WARETWIN_PUBLIC_ORIGINS': 'https://robot-hmi.example.test',
            'WARETWIN_TRUSTED_PROXY_CIDRS': '127.0.0.1/32,::1/128',
            'BACKEND_HOST': '0.0.0.0',
        })
        try:
            self.assert_prepare_rejected(runtime, 'remote-bind-state', ValueError,
                                         'PROTECTED_LAN requires backend_host')
        finally:
            if getattr(runtime, 'owns_lock', False):
                runtime.lock.close()

    def test_requested_backend_port_conflict_is_not_taken_over(self):
        with socket.socket() as blocker:
            blocker.bind(('127.0.0.1', 0))
            blocked_port = blocker.getsockname()[1]
            runtime = self.create_runtime('port-conflict', {'BACKEND_PORT': str(blocked_port)})
            try:
                self.assert_prepare_rejected(runtime, 'port-conflict-state', OSError,
                                             '.*')
            finally:
                if getattr(runtime, 'owns_lock', False):
                    runtime.lock.close()
            self.assertEqual(blocker.getsockname()[1], blocked_port)

    def test_shutdown_signals_only_the_owned_process_group(self):
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                                 start_new_session=True, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        stop_group(child)
        self.assertIsNotNone(child.returncode)


if __name__ == '__main__':
    unittest.main()
