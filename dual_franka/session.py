"""Paired episode lifecycle and sequential taught reset paths."""

import hashlib
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

from dual_franka.capture.recorder import Recorder
from dual_franka.control.reset import Reset
from dual_franka.control.reset_sequence import load_sequence, reset_segment


class ButtonEdges:
    def __init__(self):
        self.identity = None
        self.previous = set()

    def update(self, frame):
        if frame is None:
            self.identity, self.previous = None, set()
            return set()
        identity = frame["source"], frame["epoch"]
        buttons = frame.get("controller_buttons", frame["controllers"])
        pressed = set()
        for key, hand, field in [
            ("A", "right", "select"),
            ("X", "left", "select"),
            ("Y", "left", "secondary"),
        ]:
            if buttons.get(hand) and buttons[hand].get(field):
                pressed.add(key)
        edges = pressed - self.previous if identity == self.identity else set()
        self.identity, self.previous = identity, pressed
        return edges


class DualReset:
    """Reset one arm at a time; never infer collision-free paths from endpoints."""

    def __init__(self, node, arms, sequences, settings, execute):
        self.arms, self.execute = arms, execute
        self.steps = []
        for hand, arm in arms.items():
            sequence = load_sequence(sequences[hand])
            for pose in sequence["poses"]:
                reset_segment(pose["q"], pose["q"], arm.planner.model, settings["reset_speed"])
            self.steps.append(
                (
                    arm,
                    Reset(
                        node,
                        arm.planner.model,
                        sequence,
                        settings["reset_speed"],
                        settings["disable_reset_path_tolerance"],
                        arm.config.namespace,
                    ),
                )
            )
        self.index = 0
        self.opening = False

    def start(self, now):
        self.index = 0
        self.opening = self.execute
        if self.execute:
            for arm in self.arms.values():
                arm.gripper.command(False)

    def tick(self, now):
        if not self.execute:
            return True
        if self.opening:
            if any(arm.gripper.busy for arm in self.arms.values()):
                return False
            self.opening = False
            self.steps[0][1].closed = False
            self.steps[0][1].start(now)
        arm, reset = self.steps[self.index]
        if not reset.tick(arm.state.q, arm.state.dq, now):
            return False
        self.index += 1
        if self.index == len(self.steps):
            return True
        self.steps[self.index][1].closed = False
        self.steps[self.index][1].start(now)
        return False

    def cancel(self):
        for _, reset in self.steps:
            reset.cancel()


class Session:
    def __init__(self, node, config, arms, sink, cameras, output, execute, teleop_only=False):
        self.node, self.config, self.arms, self.sink = node, config, arms, sink
        self.cameras, self.output, self.execute = cameras, Path(output), execute
        self.phase = "teleoperation" if teleop_only else "unprepared"
        self.detail = (
            "Click right thumbstick to establish both references"
            if teleop_only
            else "Enter a task and prepare a session"
        )
        self.recorder = self.reset = None
        self.edges = ButtonEdges()
        self.prompts = []
        self.stop_requested_ns = None
        self.stopping_at = 0.0
        self.outcome = None

    def prepare(self, name, prompt, reset_paths):
        if self.phase not in ("unprepared", "closed"):
            return
        self.prompts = [text.strip() for text in prompt.split(";") if text.strip()]
        if not self.prompts:
            raise ValueError("Enter a task description")
        if any(not reset_paths.get(hand) for hand in self.arms):
            raise ValueError("Teach/select a reset sequence for each arm first")
        reset = DualReset(self.node, self.arms, reset_paths, self.config.control, self.execute)
        root = Path(__file__).resolve().parent
        metadata = {
            "schema_version": 2,
            "embodiment": "dual_franka_panda",
            "arms": {},
            "prompts": self.prompts,
            "preview": not self.execute,
            "control_settings": self.config.control,
            "clock_pair": {"monotonic_ns": time.monotonic_ns(), "unix_ns": time.time_ns()},
            "timing_note": "One host clock; ROS/source clocks retained. Publication is not controller acceptance.",
            "precision_control": "Hold right B to scale both arms; independent grip/trigger; right thumbstick sets both references",
            "camera_serials": self.config.cameras,
            "source_sha256": {
                str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in root.rglob("*.py")
            },
        }
        for hand, arm in self.arms.items():
            metadata["arms"][hand] = {
                **{k: str(v) if isinstance(v, Path) else v for k, v in asdict(arm.config).items()},
                "controller_parameters": arm.parameters,
                "reset_sequence": load_sequence(reset_paths[hand]),
                "pose_source": "URDF TCP FK from measured joint state",
            }
        self.recorder = Recorder(self.output, metadata, name=name)
        for hand, arm in self.arms.items():
            (self.recorder.root / f"{hand}_robot.urdf").write_text(arm.xml)
        for arm, action in reset.steps:
            action.on_command = lambda kind, msg, arm=arm: self.recorder.record_session(
                f"{arm.config.hand}_reset_{kind}", self._message(msg)
            )
        self.reset = reset
        self.sink.attach(self.recorder)
        self.cameras.restart()
        self.phase = "needs_reset"
        self.detail = "A: open grippers and follow both reset paths"

    @staticmethod
    def _message(message):
        from rosidl_runtime_py.convert import message_to_ordereddict

        return message_to_ordereddict(message)

    def stationary(self):
        return all(
            np.max(np.abs(arm.state.dq)) < 0.03 and not arm.gripper.busy
            for arm in self.arms.values()
        )

    def begin_reset(self, now):
        if not self.stationary():
            self.detail = "Wait for both arms and grippers to stop, then press A"
            return
        for arm in self.arms.values():
            arm.mapper.suspend(arm.measured)
            arm.mapper.ready = False
        self.reset.start(now)
        self.phase = "resetting"
        self.detail = "Opening grippers and resetting left, then right"

    def request_stop(self, outcome, now):
        if self.phase != "recording":
            return
        self.outcome = outcome
        self.stop_requested_ns = time.monotonic_ns()
        self.stopping_at = now
        self.phase = "stopping"
        self.detail = "Stopping both arms before reset"
        self.sink.record("events", {"event": "stop_requested", "outcome": outcome})
        for arm in self.arms.values():
            arm.mapper.suspend(arm.measured)

    def tick(self, frame, now, commands):
        edges = self.edges.update(frame)
        for command in commands:
            kind = command["kind"]
            if kind == "prepare":
                try:
                    self.prepare(command["name"], command["prompt"], command["reset_paths"])
                except (ValueError, OSError, KeyError) as error:
                    self.detail = str(error)
            elif kind in ("A", "X", "Y"):
                edges.add(kind)
            elif kind == "close_session" and self.phase != "recording":
                self.close("Session closed")
        if self.recorder is not None:
            self.recorder.health()
        if self.phase == "teleoperation":
            return True
        if self.phase == "recording":
            outcome = (
                "interrupted"
                if "Y" in edges
                else "failure"
                if "X" in edges
                else "success"
                if "A" in edges
                else None
            )
            if outcome:
                self.request_stop(outcome, now)
            return self.phase == "recording"
        if self.phase == "stopping":
            duration = max(arm.planner.horizon for arm in self.arms.values())
            if now - self.stopping_at < duration or not self.stationary():
                if now - self.stopping_at > 10:
                    raise RuntimeError("Both arms did not stop in time; reset not started")
                return False
            self.recorder.end(self.outcome, boundary_ns=self.stop_requested_ns)
            self.begin_reset(now)
            return False
        if self.phase == "resetting":
            if self.reset.tick(now):
                for arm in self.arms.values():
                    arm.planner.reset()
                    arm.mapper.suspend(arm.measured)
                    arm.mapper.ready = False
                    arm.mapper.reset_armed = False
                    if self.execute:
                        arm.gripper.toggle.closed = False
                self.phase = "reference"
                self.detail = (
                    "Release grips/triggers, then click right thumbstick to set both references"
                )
            return False
        if self.phase == "needs_reset":
            if "A" in edges:
                self.begin_reset(now)
            return False
        if self.phase in ("reference", "ready"):
            ready = all(arm.mapper.ready for arm in self.arms.values())
            self.phase = "ready" if ready else "reference"
            self.detail = (
                "A: start paired recording"
                if ready
                else "Click right thumbstick to establish both references"
            )
            if ready and "A" in edges:
                timing = {} if frame is None else frame.get("timing", {})
                if (
                    timing.get("host_offset_ns") is None
                    or timing.get("sync_age_ms", float("inf")) > 15000
                ):
                    self.detail = "Waiting for Quest clock synchronization"
                    return False
                self.recorder.start(
                    self.prompts[0],
                    {
                        "prompts": self.prompts,
                        "cameras": dict(self.cameras.metadata),
                        "arms": {
                            hand: {
                                "basis": arm.mapper.basis.tolist(),
                                "workspace_origin": arm.mapper.origin.tolist(),
                            }
                            for hand, arm in self.arms.items()
                        },
                    },
                )
                self.stop_requested_ns = None
                self.outcome = None
                self.phase = "recording"
                self.detail = "Recording both arms: A success, X failure, Y interrupted"
                for arm in self.arms.values():
                    arm.mapper.suspend(arm.measured)
                self.sink.record("events", {"event": "start", "quest": frame})
            return False
        return False

    def close(self, reason):
        if self.reset is not None:
            self.reset.cancel()
        if self.recorder is not None:
            self.recorder.end("interrupted", reason, self.stop_requested_ns)
        self.sink.close()
        self.recorder = None
        self.phase = "closed"
        self.detail = reason
