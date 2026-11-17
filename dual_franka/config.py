"""Load the two robot identities and shared control/camera settings."""

import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class ArmConfig:
    hand: str
    ip: str
    namespace: str
    yaw_deg: float
    reset_sequence: Path | None

    def topic(self, relative):
        return f"/{self.namespace}/{relative.lstrip('/')}"


@dataclass
class Configuration:
    arms: dict[str, ArmConfig]
    control: dict
    cameras: dict[str, str]
    camera_width: int
    camera_height: int
    camera_fps: int
    path: Path

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser().resolve()
        data = json.loads(path.read_text())
        arms = {}
        for hand in ("left", "right"):
            entry = data["arms"][hand]
            namespace = entry["namespace"].strip("/")
            if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", namespace):
                raise ValueError("Use a distinct simple ROS namespace for each arm")
            reset = entry.get("reset_sequence")
            reset_path = None if not reset else (path.parent / Path(reset).expanduser()).resolve()
            arms[hand] = ArmConfig(
                hand, entry["ip"], namespace, float(entry.get("yaw_deg", 0)), reset_path
            )
        if arms["left"].ip == arms["right"].ip or arms["left"].namespace == arms["right"].namespace:
            raise ValueError("The two arms need distinct IP addresses and ROS namespaces")
        control = data["control"]
        validate_control(control)
        if not all(np.isfinite(arm.yaw_deg) for arm in arms.values()):
            raise ValueError("Arm yaw angles must be finite")
        return cls(
            arms,
            control,
            data.get("cameras", {}),
            data.get("camera_width", 1280),
            data.get("camera_height", 720),
            data.get("camera_fps", 30),
            path,
        )


def validate_control(control):
    positive = (
        "scale",
        "precision_scale",
        "max_speed",
        "max_angular_speed",
        "joint_speed",
        "joint_acceleration",
        "joint_jerk",
        "workspace_radius",
        "stale_seconds",
        "gripper_speed",
        "grasp_force",
        "reset_speed",
        "trajectory_horizon",
    )
    if any(not np.isfinite(control[key]) or control[key] <= 0 for key in positive):
        raise ValueError(
            "Control speed, scale, time and force settings must be finite and positive"
        )
    if control["smoothing_seconds"] < 0 or not np.isfinite(control["smoothing_seconds"]):
        raise ValueError("Smoothing must be finite and nonnegative")
    if control["trajectory_horizon"] < 0.1 or control["frame_alignment"] not in (
        "world",
        "tool",
    ):
        raise ValueError("Use a horizon of at least 0.1 s and world/tool frame alignment")
