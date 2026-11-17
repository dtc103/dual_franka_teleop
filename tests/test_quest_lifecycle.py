"""Exercise real process/Qt shutdown with a fake robot runtime."""

import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import unittest


ROOT = Path(__file__).parents[1]


def unused_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


class LifecycleTests(unittest.TestCase):
    def test_hard_crash_releases_relay_port(self):
        port = unused_port()
        process = subprocess.Popen(
            [sys.executable, "quest_relay.py", "--port", str(port), "--no-usb-setup"],
            cwd=ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            for _ in range(60):
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                        break
                except OSError:
                    time.sleep(0.05)
            else:
                self.fail("Test relay never started")
            process.kill()
            process.wait(timeout=5)
            from dual_franka.quest.reader import WebXRReader

            reader = WebXRReader(port=port)
            reader.stop()
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate(timeout=5)

    def test_gui_interrupt_and_terminate_release_relay_port(self):
        spec = importlib.util.find_spec("PySide6")
        if spec is None:
            self.skipTest("PySide6 not installed")
        qt = Path(next(iter(spec.submodule_search_locations))) / "Qt/lib"
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
        env["LD_LIBRARY_PATH"] = str(qt) + ":" + env.get("LD_LIBRARY_PATH", "")
        script = r"""
import os, signal, socket, sys, tempfile, threading, urllib.request
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from dual_franka import application
from dual_franka.ui.window import CollectionWindow

issues = []

class FakeRuntime:
    def __init__(self, config, reader, output, execute, teleop_only):
        self.config, self.reader, self.output = config, reader, output
        self.execute, self.teleop_only = execute, teleop_only
        self.stop_event = threading.Event()
        self.status = {'phase':'connecting', 'detail':'Fake robot', 'arms':{}}
        self.cameras = SimpleNamespace(latest={}, received={}, errors={})
        self.command = Mock()
        self.thread = None
    def start(self):
        if not any(window.isVisible() for window in QApplication.topLevelWidgets()):
            issues.append('Robot startup began before the GUI was visible')
        self.thread = threading.Thread(target=lambda: self.stop_event.wait())
        self.thread.start()

class TestWindow(CollectionWindow):
    def __init__(self, *args):
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with http.open(f'http://127.0.0.1:{port}/health', timeout=2) as response:
            if response.status != 200:
                issues.append('Quest server unavailable during GUI construction')
        super().__init__(*args)
        QTimer.singleShot(150, lambda: os.kill(os.getpid(), int(os.environ['QUEST_TEST_SIGNAL'])))

port = int(sys.argv[2])
config = application.ROOT/'config/robots.json'
args = ['teleoperate.py', '--config', str(config), '--port', str(port), '--no-usb-setup']
with tempfile.TemporaryDirectory() as directory, \
     patch.dict(sys.modules, {'dual_franka.runtime':SimpleNamespace(Runtime=FakeRuntime)}), \
     patch('dual_franka.ui.window.CollectionWindow', TestWindow), \
     patch.object(sys, 'argv', args), patch.object(application, 'ROOT', Path(directory)):
    result = application.main(teleop=True)
with socket.socket() as connection:
    connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    connection.bind(('127.0.0.1', port))
if issues:
    raise RuntimeError('; '.join(issues))
if result:
    raise RuntimeError('Application returned failure')
print('GUI shutdown released server port')
"""
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=signum):
                env["QUEST_TEST_SIGNAL"] = str(signum)
                result = subprocess.run(
                    [sys.executable, "-c", script, str(signum), str(unused_port())],
                    cwd=ROOT,
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=12,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("GUI shutdown released server port", result.stdout)
