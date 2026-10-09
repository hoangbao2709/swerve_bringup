from functools import partial
from pathlib import Path
import tempfile
from threading import Thread
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen

from waretwin.runtime.static_server import BoundedThreadingHTTPServer, Handler


class StaticServerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='waretwin-static-test-')
        self.root = Path(self.temp.name) / 'frontend'
        self.root.mkdir()
        (self.root / 'index.html').write_text('<!doctype html><html>spa</html>')
        (self.root / 'assets').mkdir()
        (self.root / 'assets' / 'app.js').write_text('console.log("ok")')
        self.server = BoundedThreadingHTTPServer(
            ('127.0.0.1', 0), partial(Handler, directory=str(self.root), config={'test': True}))
        self.thread = Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temp.cleanup()

    def test_spa_refresh_assets_runtime_config_and_security_headers(self):
        with urlopen(self.base + '/robot/localization') as response:
            self.assertIn(b'<html>spa</html>', response.read())
            self.assertEqual(response.headers['X-Content-Type-Options'], 'nosniff')
            self.assertEqual(response.headers['X-Frame-Options'], 'DENY')
        with urlopen(self.base + '/assets/app.js') as response:
            self.assertIn('javascript', response.headers['Content-Type'])
            self.assertIn('immutable', response.headers['Cache-Control'])
        with urlopen(self.base + '/runtime-config.js') as response:
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            self.assertIn(b'window.WARETWIN_CONFIG=', response.read())

    def test_missing_assets_and_path_traversal_do_not_fall_back_or_escape_root(self):
        with self.assertRaises(HTTPError) as missing:
            urlopen(self.base + '/assets/missing.js')
        self.assertEqual(missing.exception.code, 404)

        outside = Path(self.temp.name) / 'outside.txt'
        outside.write_text('must-not-leak')
        with self.assertRaises(HTTPError) as traversal:
            urlopen(self.base + '/%2e%2e/outside.txt')
        self.assertIn(traversal.exception.code, (403, 404))
        self.assertNotIn(b'must-not-leak', traversal.exception.read())


if __name__ == '__main__':
    unittest.main()
