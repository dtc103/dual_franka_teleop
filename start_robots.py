#!/usr/bin/env python3
"""Start the two Panda drivers. --execute is required for hardware connections."""

import argparse

from dual_franka.config import ROOT, Configuration
from dual_franka.driver import launch_drivers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=ROOT / "config/robots.json")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--execute", action="store_true")
    group.add_argument(
        "--fake", action="store_true", help="Fake hardware only; never connects to either robot"
    )
    parser.add_argument(
        "--state-only",
        action="store_true",
        help="Read states for hand-guided teaching; do not load a motion controller",
    )
    args = parser.parse_args()
    return launch_drivers(
        Configuration.load(args.config), fake=args.fake, state_only=args.state_only
    )


if __name__ == "__main__":
    raise SystemExit(main())
