"""Local WebXR controller relay. No APK installation and no robot commands."""

import copy
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dual_franka.quest.diagnostics import ConnectionJournal

STATIC = Path(__file__).parent / "static"
# Same output axis convention as the existing reader, applied to world pose.
ROS_FROM_OPENXR = Rotation.from_quat([0.5, -0.5, -0.5, 0.5]).as_matrix()


def validate_frame(data, allowed_spaces=("local-floor",)):
    if not isinstance(data, dict) or data.get("reference_space") not in allowed_spaces:
        raise ValueError("Unsupported reference space; viewer/head-relative poses are rejected")
    source = data.get("source")
    if not isinstance(source, str) or not source or len(source) > 128:
        raise ValueError("Invalid source identifier")
    for key in ("sequence", "epoch"):
        if type(data.get(key)) is not int or data[key] < 0:
            raise ValueError(f"Invalid {key}")
    controllers = data.get("controllers")
    if not isinstance(controllers, dict) or set(controllers) != {"left", "right"}:
        raise ValueError("Expected left and right controller entries")
    result = {
        "source": source,
        "sequence": data["sequence"],
        "epoch": data["epoch"],
        "reference_space": data["reference_space"],
        "controllers": {},
    }
    if "controller_buttons" in data:
        entries = data["controller_buttons"]
        if not isinstance(entries, dict) or set(entries) != {"left", "right"}:
            raise ValueError("Expected left and right button entries")
        result["controller_buttons"] = {}
        for hand, buttons in entries.items():
            if buttons is None:
                result["controller_buttons"][hand] = None
                continue
            if (
                not isinstance(buttons, dict)
                or buttons.get("input_type") != "controller"
                or any(type(buttons.get(key)) is not bool for key in ("select", "secondary"))
            ):
                raise ValueError("Invalid physical controller buttons")
            result["controller_buttons"][hand] = {
                key: buttons[key] for key in ("input_type", "select", "secondary")
            }
    if "timing" in data:
        timing = data["timing"]
        if not isinstance(timing, dict):
            raise ValueError("Invalid timing")
        clean_timing = {}
        for key in (
            "frame_time_ms",
            "sample_time_ms",
            "time_origin_ms",
            "host_offset_ns",
            "round_trip_ms",
            "sync_age_ms",
        ):
            value = timing.get(key)
            if value is not None:
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f"Invalid timing {key}")
                clean_timing[key] = value
        result["timing"] = clean_timing
    for hand, pose in controllers.items():
        if pose is None:
            result["controllers"][hand] = None
            continue
        if not isinstance(pose, dict) or pose.get("input_type") != "controller":
            raise ValueError("Bare-hand input is not accepted")
        clean = {"input_type": "controller"}
        for key, size in (("position", 3), ("orientation", 4)):
            vector = pose.get(key)
            if (
                not isinstance(vector, list)
                or len(vector) != size
                or any(
                    type(value) not in (float, int) or not math.isfinite(value) for value in vector
                )
            ):
                raise ValueError(f"Invalid {key}")
            clean[key] = vector.copy()
        norm = math.hypot(*clean["orientation"])
        if not 0.9 <= norm <= 1.1:
            raise ValueError("Invalid orientation quaternion")
        clean["orientation"] = [value / norm for value in clean["orientation"]]
        trigger = pose.get("trigger")
        if type(trigger) not in (float, int) or not math.isfinite(trigger) or not 0 <= trigger <= 1:
            raise ValueError("Invalid trigger value")
        if type(pose.get("select")) is not bool:
            raise ValueError("Invalid controller selection button")
        clean.update(trigger=trigger, select=pose["select"])
        # Optional for older visualization/native clients; teleoperation requires it.
        if "squeeze" in pose:
            squeeze = pose["squeeze"]
            if (
                type(squeeze) not in (float, int)
                or not math.isfinite(squeeze)
                or not 0 <= squeeze <= 1
            ):
                raise ValueError("Invalid squeeze value")
            clean["squeeze"] = squeeze
        if "secondary" in pose:
            if type(pose["secondary"]) is not bool:
                raise ValueError("Invalid secondary button")
            clean["secondary"] = pose["secondary"]
        if "thumbstick" in pose:
            if type(pose["thumbstick"]) is not bool:
                raise ValueError("Invalid thumbstick button")
            clean["thumbstick"] = pose["thumbstick"]
        if "axes" in pose:
            axes = pose["axes"]
            if (
                not isinstance(axes, list)
                or len(axes) > 16
                or any(
                    type(x) not in (float, int) or not math.isfinite(x) or not -1 <= x <= 1
                    for x in axes
                )
            ):
                raise ValueError("Invalid axes")
            clean["axes"] = axes.copy()
        if "buttons" in pose:
            buttons = pose["buttons"]
            if not isinstance(buttons, list) or len(buttons) > 16:
                raise ValueError("Invalid buttons")
            clean["buttons"] = []
            for button in buttons:
                if (
                    not isinstance(button, dict)
                    or type(button.get("value")) not in (int, float)
                    or not math.isfinite(button["value"])
                    or not 0 <= button["value"] <= 1
                    or type(button.get("pressed")) is not bool
                    or type(button.get("touched")) is not bool
                ):
                    raise ValueError("Invalid button")
                clean["buttons"].append({k: button[k] for k in ("value", "pressed", "touched")})
        # An inferred position is not presented as an actively tracked pose.
        result["controllers"][hand] = None if pose.get("emulatedPosition") else clean
    return result


class TrackingState:
    def __init__(self, stale_seconds=0.5, allowed_spaces=("local-floor",)):
        self.lock = threading.Lock()
        self.frame = None
        self.received = 0
        self.stale_seconds = stale_seconds
        self.allowed_spaces = allowed_spaces
        self.on_frame = None
        self.accepted = 0
        self.browser = {}
        self.browser_at = 0

    def publish(self, data):
        receipt_ns = time.monotonic_ns()
        frame = validate_frame(data, self.allowed_spaces)
        frame["received_ns"] = receipt_ns
        with self.lock:
            if self.frame is not None:
                if frame["source"] != self.frame["source"]:
                    if time.monotonic() - self.received < self.stale_seconds:
                        raise ValueError("Another Quest browser is already streaming")
                elif frame["sequence"] <= self.frame["sequence"]:
                    raise ValueError("Out-of-order frame")
            self.frame, self.received = frame, time.monotonic()
            self.accepted += 1
            callback = self.on_frame
        if callback is not None:
            callback(copy.deepcopy(frame), receipt_ns)

    def snapshot(self):
        with self.lock:
            if self.frame is None or time.monotonic() - self.received > self.stale_seconds:
                return None
            return copy.deepcopy(self.frame)

    def health(self):
        with self.lock:
            age = None if self.frame is None else time.monotonic() - self.received
            fresh = age is not None and age <= self.stale_seconds
            return {
                "accepted_frames": self.accepted,
                "active_source": None if self.frame is None else self.frame["source"],
                "frame_age_ms": None if age is None else round(age * 1000),
                "streaming": fresh,
                "buttons": {
                    hand: {
                        key: pose.get(key)
                        for key in ("thumbstick", "squeeze", "trigger", "select", "secondary")
                    }
                    for hand, pose in self.frame["controllers"].items()
                    if pose
                }
                if fresh
                else {},
                "tracked": [hand for hand, pose in self.frame["controllers"].items() if pose]
                if fresh
                else [],
                "browser": copy.deepcopy(self.browser),
                "browser_age_seconds": None
                if not self.browser_at
                else round(time.monotonic() - self.browser_at, 1),
            }


class RelayServer(ThreadingHTTPServer):
    # Chromium may open idle/preconnected sockets while also fetching scripts.
    # Bound socket lifetime and leave enough listen backlog for those bursts.
    request_queue_size = 64
    daemon_threads = True

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(3)
        return connection, address


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, status, content, mime="text/plain"):
        try:
            self._send_reply(status, content, mime)
        except (BrokenPipeError, ConnectionResetError):
            pass  # Browser cancelled a request or reloaded the page.

    def _send_reply(self, status, content, mime):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(content)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/health":
            self.reply(200, json.dumps(self.server.state.health()).encode(), "application/json")
            return
        if path == "/clock":
            self.reply(
                200,
                json.dumps({"host_monotonic_ns": time.monotonic_ns()}).encode(),
                "application/json",
            )
            return
        files = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "application/javascript"),
        }
        if path not in files:
            self.reply(404, b"Not found")
            return
        name, mime = files[path]
        self.server.journal.event(
            "http_asset", path=path, user_agent=self.headers.get("User-Agent", "")
        )
        self.reply(200, (STATIC / name).read_bytes(), mime)

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        if self.path not in ("/pose", "/client-status"):
            self.reply(404, b"Not found")
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 32768:
                raise ValueError("Invalid message size")
            data = json.loads(self.rfile.read(size))
            if self.path == "/pose":
                self.server.state.publish(data)
            else:
                if not isinstance(data, dict):
                    raise ValueError("Expected browser status object")
                fields = (
                    "client",
                    "source",
                    "session",
                    "visibility",
                    "error",
                    "inputs",
                    "last_frame_age_ms",
                    "last_ack_age_ms",
                )
                status = {key: data.get(key) for key in fields}
                with self.server.state.lock:
                    self.server.state.browser = status
                    self.server.state.browser_at = time.monotonic()
                self.server.journal.event(
                    "browser_status",
                    **{
                        key: status[key]
                        for key in ("client", "session", "visibility", "error", "inputs")
                    },
                )
            self.reply(204, b"")
        except (ValueError, TypeError, KeyError) as error:
            self.server.journal.event("request_rejected", path=self.path, detail=str(error))
            self.reply(400, str(error).encode())
        except (TimeoutError, ConnectionResetError, BrokenPipeError):
            self.server.journal.event("request_disconnected", path=self.path)


class WebXRReader:
    def __init__(self, host="127.0.0.1", port=8443, log_path=None):
        self.state = TrackingState()
        self.journal = ConnectionJournal(log_path)
        self.server = RelayServer((host, port), Handler)
        self.server.journal = self.journal
        self.server.state = self.state
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.journal.event("server_started", host=host, port=self.server.server_port)
        print(f"WebXR relay started on {host}:{self.server.server_port}", flush=True)
        print(f"In another terminal: adb reverse tcp:{port} tcp:{port}", flush=True)
        print(
            f"In the Quest browser: http://localhost:{port}/ -> Start controller tracking",
            flush=True,
        )

    def snapshot(self):
        return self.state.snapshot()

    def get_frame_epoch(self):
        frame = self.snapshot()
        return None if frame is None else (frame["source"], frame["epoch"])

    def get_hand_controller_transform_openxr(self, hand):
        frame = self.snapshot()
        pose = None if frame is None else frame["controllers"].get(hand)
        if pose is None:
            return None
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(pose["orientation"]).as_matrix()
        matrix[:3, 3] = pose["position"]
        return matrix

    def get_hand_controller_transform_ros(self, hand):
        matrix = self.get_hand_controller_transform_openxr(hand)
        if matrix is None:
            return None
        conversion = np.eye(4)
        conversion[:3, :3] = ROS_FROM_OPENXR
        return conversion @ matrix

    def get_trigger_value(self, hand):
        frame = self.snapshot()
        pose = None if frame is None else frame["controllers"].get(hand)
        return 0.0 if pose is None else pose["trigger"]

    def get_button_state(self, button):
        hand = {"X": "left", "A": "right"}.get(button)
        frame = self.snapshot()
        pose = None if frame is None else frame["controllers"].get(hand)
        return False if pose is None else pose["select"]

    def stop(self):
        self.journal.event("server_stopping")
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
