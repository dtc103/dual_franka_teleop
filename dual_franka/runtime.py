"""One ROS worker for two arms, one Quest snapshot and one episode clock."""

import queue
import threading
import time

from dual_franka.capture.sink import RecordingSink


class Runtime:
    def __init__(self, config, reader, output, execute=False, teleop_only=False):
        self.config, self.reader, self.output = config, reader, output
        self.execute, self.teleop_only = execute, teleop_only
        self.commands = queue.SimpleQueue()
        self.stop_event = threading.Event()
        self.status = {
            "phase": "connecting",
            "detail": "Connecting to both robot drivers",
            "arms": {},
        }
        self.sink = RecordingSink()
        self.cameras = self._new_cameras()
        self.arms = {}
        self.session = None
        self.thread = None

    def _new_cameras(self):
        if self.teleop_only:
            return None
        from dual_franka.capture.manager import CameraManager

        return CameraManager(self.config, self.sink)

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        if self.thread is not None:
            # A failed worker has already closed ROS, cameras and the recorder.
            # Reconnect with fresh state; never reuse a previous motion target.
            self.sink = RecordingSink()
            self.cameras = self._new_cameras()
            self.arms = {}
            self.session = None
            self.commands = queue.SimpleQueue()
        self.stop_event.clear()
        self.status = {
            "phase": "connecting",
            "detail": "Connecting to both robot drivers",
            "arms": {},
        }
        self.thread = threading.Thread(target=self.run, name="dual-control", daemon=False)
        self.thread.start()

    def command(self, kind, **values):
        self.commands.put({"kind": kind, **values})

    def drain_commands(self):
        result = []
        while not self.commands.empty():
            result.append(self.commands.get())
        return result

    def publish_status(self):
        session = self.session
        counts = {"total": 0, "success": 0, "failure": 0, "interrupted": 0}
        if session.recorder is not None:
            finished = list(session.recorder.finished.copy().values())
            counts["total"] = len(finished)
            for outcome in ("success", "failure", "interrupted"):
                counts[outcome] = sum(item["outcome"] == outcome for item in finished)
        self.status = {
            "phase": session.phase,
            "detail": session.detail,
            "location": "" if session.recorder is None else str(session.recorder.root),
            "counts": counts,
            "arms": {
                hand: {
                    "status": arm.mapper.status,
                    "ready": arm.mapper.ready,
                    "engaged": arm.mapper.engaged,
                    "scale": arm.mapper.scale,
                    "width": arm.width,
                    "target": arm.target[:3, 3].tolist(),
                }
                for hand, arm in self.arms.items()
            },
        }

    def run(self):
        rclpy = node = None
        reason = "Closed by operator"
        try:
            # Keep ROS/IK imports and hardware startup out of GUI construction.
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from dual_franka.control.arm import Arm
            from dual_franka.session import Session

            if self.stop_event.is_set():
                return
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node("dual_franka_collection")
            if self.cameras is not None:
                self.cameras.start()
            for hand, config in self.config.arms.items():
                arm = Arm(node, config, self.config.control, self.sink, self.execute)
                self.arms[hand] = arm
                arm.initialize()
            self.session = Session(
                node,
                self.config,
                self.arms,
                self.sink,
                self.cameras,
                self.output,
                self.execute,
                self.teleop_only,
            )
            self.reader.state.on_frame = lambda frame, receipt: self.sink.record(
                "quest", frame, receipt
            )
            previous = clock_at = time.monotonic()
            while rclpy.ok() and not self.stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.002)
                now = time.monotonic()
                if now - previous < 0.02:
                    continue
                for arm in self.arms.values():
                    arm.health(now)
                    if now - previous > 0.2:
                        arm.mapper.suspend(arm.measured)
                frame = self.reader.snapshot()
                phase = self.session.phase
                allow_input = self.session.tick(frame, now, self.drain_commands())
                resetting = phase == "resetting" or self.session.phase == "resetting"
                if not resetting:
                    # Compute both trajectories from the same input snapshot/time,
                    # then publish. Separate robot controllers are not hardware-locked.
                    updates = {
                        hand: arm.step(frame, now - previous, now, allow_input)
                        for hand, arm in self.arms.items()
                    }
                    samples = {hand: sample for hand, (_, sample) in updates.items()}
                    self.sink.record(
                        "control", {"command_time_ns": time.monotonic_ns(), "arms": samples}
                    )
                    for hand, (segment, sample) in updates.items():
                        self.sink.record(f"{hand}_control", sample)
                        self.arms[hand].send(segment)
                if now - clock_at >= 1:
                    self.sink.record(
                        "clock_samples",
                        {
                            "host_monotonic_ns": time.monotonic_ns(),
                            "host_unix_ns": time.time_ns(),
                            "ros_ns": node.get_clock().now().nanoseconds,
                        },
                    )
                    clock_at = now
                self.publish_status()
                previous = now
        except Exception as error:
            reason = str(error)
            if self.session is not None and self.session.recorder is not None:
                self.session.recorder.end("interrupted", reason, time.monotonic_ns())
            self.status = {"phase": "fault", "detail": reason, "arms": {}}
            print(f"Dual control stopped: {reason}", flush=True)
        finally:
            self.reader.state.on_frame = None
            if self.session is not None and self.session.reset is not None:
                self.session.reset.cancel()
            for arm in self.arms.values():
                try:
                    arm.stop(time.monotonic())
                except (ValueError, RuntimeError) as error:
                    print(f"{arm.config.hand} stop: {error}", flush=True)
            # Deliver gripper/action cancellation callbacks before destroying ROS.
            if node is not None:
                until = time.monotonic() + 0.5
                while rclpy.ok() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.01)
            if self.cameras is not None:
                self.cameras.close()
            if self.session is not None:
                self.session.close(reason)
            else:
                self.sink.close()
            if node is not None:
                node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            if self.status["phase"] != "fault":
                self.status = {"phase": "closed", "detail": reason, "arms": {}}
