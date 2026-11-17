"""Start the real GUI/relay with no robot publishers and no camera devices."""

from importlib.util import find_spec
import os
from pathlib import Path
import subprocess
import sys
import unittest


class StartupTests(unittest.TestCase):
    def test_gui_and_relay_are_ready_while_robot_connections_are_pending(self):
        spec = find_spec("PySide6")
        if spec is None or find_spec("rclpy") is None:
            self.skipTest("Requires the Jazzy/PySide6 environment")
        qt = Path(next(iter(spec.submodule_search_locations))) / "Qt/lib"
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen", ROS_DOMAIN_ID="199")
        env["LD_LIBRARY_PATH"] = str(qt) + ":" + env.get("LD_LIBRARY_PATH", "")
        script = r"""
import json, os, signal, socket, sys, tempfile, time, urllib.request
from pathlib import Path
from unittest.mock import patch
from PySide6.QtCore import QTimer
from dual_franka import application
from dual_franka.ui.window import CollectionWindow

issues = []
started = None
class StartupWindow(CollectionWindow):
    def __init__(self, *args):
        super().__init__(*args)
        QTimer.singleShot(300, self.check_startup)
    def check_startup(self):
        try:
            if not self.isVisible():
                issues.append('GUI not visible before robot inputs')
            if time.monotonic() - started > 4:
                issues.append('GUI startup blocked on robot readiness')
            if any(arm.state is not None for arm in self.runtime.arms.values()):
                issues.append('Unexpected robot state publisher in isolated test')
            if self.runtime.cameras is not None or self.images:
                issues.append('Standalone teleoperation constructed camera handling or previews')
            if 'pyrealsense2' in sys.modules:
                issues.append('Standalone teleoperation imported the camera SDK')
            http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with http.open(f'http://127.0.0.1:{port}/health', timeout=1) as response:
                health = json.load(response)
            if health['streaming']:
                issues.append('Unexpected headset stream in isolated test')
            print('GUI visible and Quest HTTP ready without any robot input', flush=True)
        except Exception as error:
            issues.append(str(error))
        finally:
            os.kill(os.getpid(), signal.SIGINT)

with socket.socket() as connection:
    connection.bind(('127.0.0.1', 0))
    port = connection.getsockname()[1]
with tempfile.TemporaryDirectory() as directory:
    data = json.loads((application.ROOT/'config/robots.json').read_text())
    # Enumerate only; this deliberately impossible serial cannot reset or
    # stream any physical camera. Missing cameras must not block Qt/HTTP.
    data['cameras'] = {'missing': '__absent_startup_test_camera__'}
    config = Path(directory)/'robots.json'
    config.write_text(json.dumps(data))
    args = ['teleoperate.py', '--config', str(config), '--port', str(port), '--no-usb-setup']
    with patch.object(sys, 'argv', args), patch.object(application, 'ROOT', Path(directory)), \
         patch('dual_franka.ui.window.CollectionWindow', StartupWindow):
        started = time.monotonic()
        application.main(teleop=True)  # No --execute, plus isolated ROS domain.
if issues:
    raise RuntimeError('; '.join(issues))
"""
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).parents[1],
            env=env,
            capture_output=True,
            text=True,
            timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("GUI visible and Quest HTTP ready without any robot input", result.stdout)
