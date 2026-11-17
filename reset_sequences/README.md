# Taught reset sequences

Run `teach_reset.py` to save `left.json` and `right.json` here. These contain
seven joint angles in radians per waypoint; the final waypoint is the start
pose. The teaching script updates `config/robots.json` with the selected paths.

No robot poses are invented or supplied by default. Standalone teleoperation
uses measured current poses and does not execute these reset sequences.
