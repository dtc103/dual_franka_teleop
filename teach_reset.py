#!/usr/bin/env python3
"""Read paired reset waypoints; no arm or gripper commands are sent.

Hand-guide with motion controllers inactive. Save clearance points in travel
order; the last pair is the episode starting pose. Reset executes left's full
path first, then right's full path, so teach with that ordering in mind.
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from dual_franka.capture.recorder import atomic_json
from dual_franka.config import ROOT, Configuration
from dual_franka.control.panda import JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/robots.json")
    parser.add_argument("--output", type=Path, default=ROOT / "reset_sequences")
    args = parser.parse_args()
    config = Configuration.load(args.config)
    import rclpy
    from franka_msgs.msg import FrankaState
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node("teach_dual_reset")
    states = {}

    def receive(hand, message):
        states[hand] = (message, time.monotonic())

    # Node retains the subscriptions for the lifetime of this read-only tool.
    [
        node.create_subscription(
            FrankaState,
            arm.topic("franka_robot_state_broadcaster/robot_state"),
            lambda msg, hand=hand: receive(hand, msg),
            qos_profile_sensor_data,
        )
        for hand, arm in config.arms.items()
    ]
    points = {hand: [] for hand in config.arms}
    args.output.mkdir(parents=True, exist_ok=True)
    print(
        "Read-only teaching: S save pair · D delete last pair · Q finish. Last pair is the start pose."
    )
    try:
        while True:
            key = (
                input("Position both arms, release guiding controls, then S/D/Q: ").strip().lower()
            )
            if key == "q":
                break
            if key == "d":
                for poses in points.values():
                    if poses:
                        poses.pop()
                print(f"{len(points['left'])} pairs kept")
                continue
            if key != "s":
                continue
            # Drain buffered samples and collect fresh state after user input.
            states.clear()
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
            if set(states) != set(config.arms) or any(
                time.monotonic() - t > 0.25 for _, t in states.values()
            ):
                print("Not saved: missing fresh state from one or both robot drivers")
                continue
            if any(np.max(np.abs(state.dq)) > 0.03 for state, _ in states.values()):
                print("Not saved: an arm is still moving")
                continue
            if any(not np.isfinite(state.q).all() for state, _ in states.values()):
                print("Not saved: joint data is invalid")
                continue
            for hand, (state, _) in states.items():
                points[hand].append({"q": list(state.q)})
            print(f"Saved pair {len(points['left'])}")
    finally:
        if points["left"]:
            data = json.loads(config.path.read_text())
            for hand, poses in points.items():
                path = (args.output / f"{hand}.json").resolve()
                atomic_json(path, {"joints": JOINTS, "poses": poses})
                data["arms"][hand]["reset_sequence"] = str(path)
                print(f"Wrote {path}")
            atomic_json(config.path, data)
            print("Default reset paths updated in the configuration.")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
