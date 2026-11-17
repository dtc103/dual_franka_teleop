"""Verify fault-state GUI controls in an isolated offscreen Qt process."""

from importlib.util import find_spec
import os
from pathlib import Path
import subprocess
import sys
import unittest


class FaultWindowTests(unittest.TestCase):
    def test_fault_remains_interactive_and_reconnect_is_explicit(self):
        spec = find_spec("PySide6")
        if spec is None:
            self.skipTest("PySide6 not installed")
        qt = Path(next(iter(spec.submodule_search_locations))) / "Qt/lib"
        env = os.environ.copy()
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["LD_LIBRARY_PATH"] = str(qt) + ":" + env.get("LD_LIBRARY_PATH", "")
        script = """
from unittest.mock import Mock
from types import SimpleNamespace
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer, QCoreApplication, QEvent
from dual_franka.config import ROOT, Configuration
from dual_franka.ui.environment import configure_qt_plugins
configure_qt_plugins()
from dual_franka.ui.window import CollectionWindow
app = QApplication([])
thread = Mock()
thread.is_alive.return_value = False
runtime = SimpleNamespace(config=Configuration.load(ROOT/'config/robots.json'), execute=False,
                          teleop_only=False, output=ROOT/'datasets', thread=thread,
                          stop_event=Mock(), command=Mock(), start=Mock(),
                          status={'phase':'fault', 'detail':'Right robot is in Reflex', 'arms':{}},
                          cameras=SimpleNamespace(latest={}, received={}, errors={}))
window = CollectionWindow(runtime, 8443)
window.show()
window.refresh()
if not window.reconnect.isEnabled() or not window.name.isEnabled():
    raise RuntimeError('Fault window disabled editing/reconnection')
if window.prepare.isEnabled():
    raise RuntimeError('Recording must remain disabled while disconnected')
window.reconnect.click()
runtime.start.assert_called_once()
window.name.setText('Operator can edit the session after a fault')
QTimer.singleShot(30, app.quit)
app.exec()
window.close()
window.deleteLater()
QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
del window
runtime.teleop_only = True
window = CollectionWindow(runtime, 8443)
window.show()
window.refresh()
if window.prepare.isVisible() or window.actions['A'].isVisible() or window.images:
    raise RuntimeError('Standalone teleoperation shows unusable recording controls')
window.close()
window.deleteLater()
QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
del window
print('Fault GUI and teleoperation GUI passed')
"""
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).parents[1],
            env=env,
            capture_output=True,
            text=True,
            timeout=15,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Fault GUI and teleoperation GUI passed", result.stdout)
