"""CLI/GUI entry point; the Quest relay lives for the entire application."""

import argparse
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

from dual_franka.config import ROOT, Configuration, validate_control
from dual_franka.quest.reader import WebXRReader
from dual_franka.quest.setup import USBConnection


def options(teleop=False):
    parser = argparse.ArgumentParser(description="Two Quest controllers → two Panda arms")
    parser.add_argument("--config", type=Path, default=ROOT / "config/robots.json")
    parser.add_argument("--output", type=Path, default=ROOT / "datasets/demonstrations")
    parser.add_argument("--port", type=int, default=8444)
    parser.add_argument("--execute", action="store_true", help="Enable live arm/gripper commands")
    parser.add_argument(
        "--headless", action="store_true", help="Standalone teleoperation without a GUI"
    )
    parser.add_argument("--no-usb-setup", action="store_true")
    parser.add_argument(
        "--quest-serial", help="Select the Quest when several Android devices are connected"
    )
    parser.add_argument(
        "--open-quest-browser",
        action="store_true",
        help="Open the local page in Quest Browser once at startup",
    )
    parser.add_argument("--scale", type=float)
    parser.add_argument("--precision-scale", type=float)
    parser.add_argument("--yaw-left", type=float)
    parser.add_argument("--yaw-right", type=float)
    for name in (
        "max_speed",
        "max_angular_speed",
        "joint_speed",
        "joint_acceleration",
        "joint_jerk",
        "smoothing_seconds",
        "workspace_radius",
        "stale_seconds",
        "trajectory_horizon",
        "grasp_force",
        "gripper_speed",
        "reset_speed",
    ):
        parser.add_argument("--" + name.replace("_", "-"), type=float)
    parser.add_argument("--frame-alignment", choices=["world", "tool"])
    parser.add_argument("--disable-reset-path-tolerance", action="store_true", default=None)
    parser.add_argument("--reset-left", type=Path)
    parser.add_argument("--reset-right", type=Path)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Port must be between 1 and 65535")
    if args.headless and not teleop:
        parser.error("Use teleoperate.py for headless control; collection uses the GUI")
    return args


def main(teleop=False):
    args = options(teleop)
    config = Configuration.load(args.config)
    import math
    from dataclasses import replace

    for key in config.control:
        value = getattr(args, key, None)
        if value is not None:
            config.control[key] = value
    validate_control(config.control)
    for hand in config.arms:
        value = getattr(args, "yaw_" + hand)
        if value is not None:
            if not math.isfinite(value):
                raise ValueError("Yaw must be finite")
            config.arms[hand] = replace(config.arms[hand], yaw_deg=value)
    for hand in config.arms:
        reset = getattr(args, "reset_" + hand)
        if reset is not None:
            config.arms[hand] = replace(
                config.arms[hand], reset_sequence=reset.expanduser().resolve()
            )
    if not args.headless:
        # This may re-exec to select Qt libraries; do it before opening sockets.
        from dual_franka.ui.environment import ensure_qt_runtime

        ensure_qt_runtime()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = ROOT / "logs" / f"quest_connection_{stamp}.jsonl"
    reader = WebXRReader(port=args.port, log_path=log_path)
    print(f"Quest connection journal: {log_path}", flush=True)
    print(
        "Quest server is listening. Robot connections are not required to open the page.",
        flush=True,
    )
    reader.state.stale_seconds = config.control["stale_seconds"]
    usb = runtime = window = None
    try:
        if not args.no_usb_setup:
            usb = USBConnection(
                args.port, reader.journal, args.quest_serial, args.open_quest_browser
            )
            usb.start()
        if not args.headless:
            from PySide6.QtCore import QTimer
            from PySide6.QtWidgets import QApplication
            from dual_franka.ui.environment import configure_qt_plugins

            configure_qt_plugins()
            app = QApplication([])
            app.setStyle("Fusion")
        from dual_franka.runtime import Runtime

        # Construction only creates state. Hardware work starts in the worker
        # after the window is shown and Qt can process its events.
        runtime = Runtime(config, reader, args.output.expanduser(), args.execute, teleop)

        def request_stop(signum, *_):
            reader.journal.event("shutdown_requested", signal=signal.Signals(signum).name)
            runtime.stop_event.set()
            if window is not None:
                window.close()

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        if args.headless:
            runtime.start()
            while runtime.thread.is_alive():
                print(runtime.status, flush=True)
                time.sleep(1)
        else:
            from dual_franka.ui.window import CollectionWindow

            window = CollectionWindow(runtime, args.port)
            window.quest_usb = usb
            window.show()

            def connect_robots():
                if not runtime.stop_event.is_set():
                    runtime.start()

            QTimer.singleShot(0, connect_robots)
            print("GUI opened. Connecting to robots in the background…", flush=True)
            result = app.exec()
            reader.journal.event("gui_event_loop_exited", exit_code=result)
    finally:
        if runtime is not None:
            runtime.stop_event.set()
            if runtime.thread is not None:
                runtime.thread.join()
        if usb:
            usb.stop()
        reader.stop()
    return 1 if runtime.status["phase"] == "fault" else 0
