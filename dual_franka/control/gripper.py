"""One asynchronous open/close action per trigger edge."""

import time

from dual_franka.control.gripper_toggle import ToggleCommands


class Gripper:
    def __init__(self, node, arm, settings, record, execute):
        from franka_msgs.action import Grasp, Move
        from rclpy.action import ActionClient

        self.arm, self.settings, self.record = arm, settings, record
        self.execute = execute
        self.move = ActionClient(node, Move, arm.topic("panda_gripper/move"))
        self.grasp = ActionClient(node, Grasp, arm.topic("panda_gripper/grasp"))
        self.toggle = ToggleCommands()
        self.pending = self.handle = self.result = None
        self.error = None
        self.closed = False
        self.started = 0.0

    @property
    def busy(self):
        return self.pending is not None or self.result is not None

    def command(self, close):
        from franka_msgs.action import Grasp, Move

        if self.busy:
            return
        if not self.execute:
            self.toggle.closed = close
            return
        client = self.grasp if close else self.move
        if not client.server_is_ready():
            raise RuntimeError(f"{self.arm.hand} gripper action server unavailable")
        goal = Grasp.Goal() if close else Move.Goal()
        goal.width, goal.speed = (0.0 if close else 0.08), self.settings["gripper_speed"]
        if close:
            goal.force = self.settings["grasp_force"]
            goal.epsilon.inner, goal.epsilon.outer = 0.005, 0.08
        self.toggle.active = close
        self.started = time.monotonic()
        self.record("gripper_command", goal)
        self.pending = client.send_goal_async(goal)
        self.pending.add_done_callback(self._accepted)

    def _accepted(self, future):
        self.pending = None
        self.handle = future.result()
        if not self.handle.accepted:
            self.error = f"{self.arm.hand} gripper command rejected"
            return
        self.result = self.handle.get_result_async()
        if self.closed:
            self.handle.cancel_goal_async()

    def tick(self, clicked=False):
        if self.error:
            raise RuntimeError(self.error)
        if self.result is not None and self.result.done():
            result = self.result.result()
            self.record("gripper_result", result)
            self.result = self.handle = None
            success = result.status == 4 and result.result.success
            self.toggle.complete(success)
            if not success:
                raise RuntimeError(f"{self.arm.hand} gripper failed: {result.result.error}")
        if self.busy and time.monotonic() - self.started > 15:
            raise RuntimeError(f"{self.arm.hand} gripper timed out")
        if clicked and not self.busy:
            self.command(not self.toggle.closed)

    def cancel(self):
        self.closed = True
        if self.handle is not None:
            self.handle.cancel_goal_async()
