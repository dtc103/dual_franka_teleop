#!/usr/bin/env python3
"""Teleoperate both arms without recording episodes or reset sequences."""

from dual_franka.application import main

if __name__ == "__main__":
    raise SystemExit(main(teleop=True))
