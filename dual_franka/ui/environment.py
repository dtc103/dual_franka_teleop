"""Keep PySide6 and the ROS environment's Qt libraries separate."""

import os
import sys
from pathlib import Path


def ensure_qt_runtime():
    """Restart before Qt import so pip PySide and Pixi Qt never mix libraries."""
    import importlib.util

    spec = importlib.util.find_spec("PySide6")
    if sys.platform != "linux" or spec is None or not spec.submodule_search_locations:
        return
    qt_lib = Path(next(iter(spec.submodule_search_locations))) / "Qt/lib"
    if not (qt_lib / "libQt6Core.so.6").is_file():
        return  # Non-wheel installations use their environment's own Qt.
    current = os.environ.get("LD_LIBRARY_PATH", "")
    if current.split(":")[0] == str(qt_lib):
        return
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = str(qt_lib) + (":" + current if current else "")
    os.execve(sys.executable, [sys.executable, *sys.argv], env)


def configure_qt_plugins():
    """Use the whole PySide plugin tree, including Wayland shell integrations."""
    import PySide6
    from PySide6.QtCore import QCoreApplication

    plugins = Path(PySide6.__file__).parent / "Qt/plugins"
    if plugins.is_dir():
        os.environ["QT_PLUGIN_PATH"] = str(plugins)
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins / "platforms")
        QCoreApplication.setLibraryPaths([str(plugins)])
