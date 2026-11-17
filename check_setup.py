#!/usr/bin/env python3
"""Read-only network, USB and installed dependency diagnostics; no robot control."""

import argparse
import socket
import subprocess
from importlib.util import find_spec

from dual_franka.config import ROOT, Configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=ROOT / "config/robots.json")
    args = parser.parse_args()
    config = Configuration.load(args.config)
    for arm in config.arms.values():
        try:
            with socket.create_connection((arm.ip, 443), timeout=2):
                print(f"{arm.hand}: {arm.ip} HTTPS reachable (does not verify FCI control)")
        except OSError as error:
            print(f"{arm.hand}: {arm.ip}: {error}")
    try:
        result = subprocess.run(["adb", "devices", "-l"], capture_output=True, text=True, timeout=5)
        print(result.stdout or result.stderr)
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"ADB unavailable: {error}")
    for module in ["rclpy", "pink", "pinocchio", "quadprog", "PySide6", "pyrealsense2", "cv2"]:
        print(f"{module}: {'installed' if find_spec(module) else 'missing'}")


if __name__ == "__main__":
    main()
