#!/usr/bin/env python3
"""Test Quest USB/browser/tracking without ROS, cameras or robot commands."""

import argparse
import signal
import threading
from datetime import datetime, timezone
from pathlib import Path

from dual_franka.quest.reader import WebXRReader
from dual_franka.quest.setup import USBConnection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--quest-serial")
    parser.add_argument("--open-quest-browser", action="store_true")
    parser.add_argument("--no-usb-setup", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Port must be between 1 and 65535")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log = Path(__file__).resolve().parent / "logs" / f"quest_connection_{stamp}.jsonl"
    reader = WebXRReader(port=args.port, log_path=log)
    usb = USBConnection(args.port, reader.journal, args.quest_serial, args.open_quest_browser)
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    if not args.no_usb_setup:
        usb.start()
    print(f"Tracking only. No robot connection. Journal: {log}", flush=True)
    try:
        while not stopped.wait(1):
            health = reader.state.health()
            reader.journal.event(
                "tracking_status", streaming=health["streaming"], tracked=health["tracked"]
            )
    finally:
        if not args.no_usb_setup:
            usb.stop()
        reader.stop()


if __name__ == "__main__":
    main()
