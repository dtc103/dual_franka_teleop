"""Asynchronous gripper-open and joint-trajectory reset sequence."""

import numpy as np

from dual_franka.control.panda import JOINTS
from dual_franka.control.reset_sequence import reset_segment


class Reset:
    def __init__(self, node, model, sequence, speed, disable_path_tolerance=False, namespace=""):
        prefix = "/" + namespace.strip("/") if namespace else ""
        from control_msgs.action import FollowJointTrajectory
        from franka_msgs.action import Move
        from rclpy.action import ActionClient

        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            prefix + "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.gripper = ActionClient(node, Move, prefix + "/panda_gripper/move")
        self.model, self.sequence, self.speed = model, sequence, speed
        self.disable_path_tolerance = disable_path_tolerance
        self.pending = self.handle = self.result = None
        self.index = -1
        self.deadline = 0
        self.active = False
        self.closed = False
        self.on_command = lambda *args: None

    def start(self, now):
        from franka_msgs.action import Move

        if not self.client.server_is_ready() or not self.gripper.server_is_ready():
            raise RuntimeError("Reset action servers unavailable")
        self.active = True
        self.index = -1
        goal = Move.Goal()
        goal.width, goal.speed = 0.08, 0.1
        self.on_command("gripper", goal)
        self.pending = self.gripper.send_goal_async(goal)
        self.pending.add_done_callback(self._accepted)
        self.deadline = now + 15

    def _accepted(self, future):
        self.pending = None
        self.handle = future.result()
        if not self.handle.accepted:
            self.active = False
            return
        self.result = self.handle.get_result_async()
        if self.closed:
            self.handle.cancel_goal_async()

    def tick(self, q, dq, now):
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from rclpy.duration import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        if not self.active:
            raise RuntimeError("Reset action rejected")
        if now > self.deadline:
            self.cancel()
            raise RuntimeError("Reset timed out; no automatic retry")
        if self.result is None or not self.result.done():
            return False
        result = self.result.result()
        self.on_command("result", result)
        self.handle = self.result = None
        success = result.status == 4 and (
            result.result.success if self.index == -1 else result.result.error_code == 0
        )
        if not success:
            raise RuntimeError(f"Reset action failed: {result.result}")
        if np.max(np.abs(dq)) > 0.03:
            raise RuntimeError("Robot still moving after reset action; reset stopped")
        self.index += 1
        if self.index == len(self.sequence["poses"]):
            self.active = False
            return True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        segment = reset_segment(q, self.sequence["poses"][self.index]["q"], self.model, self.speed)
        for seconds in np.linspace(0.0, segment.duration, max(8, int(segment.duration * 20))):
            point = JointTrajectoryPoint()
            pos, vel, acc = segment.sample(seconds)
            point.positions, point.velocities, point.accelerations = (
                pos.tolist(),
                vel.tolist(),
                acc.tolist(),
            )
            point.time_from_start = Duration(seconds=float(seconds)).to_msg()
            goal.trajectory.points.append(point)
        goal.goal_tolerance = [JointTolerance(name=name, position=0.03) for name in JOINTS]
        goal.path_tolerance = self.path_tolerances(JointTolerance)
        goal.goal_time_tolerance = Duration(seconds=5.0).to_msg()
        self.on_command("trajectory", goal)
        self.pending = self.client.send_goal_async(goal)
        self.pending.add_done_callback(self._accepted)
        self.deadline = now + segment.duration + 10
        return False

    def path_tolerances(self, tolerance_type):
        if self.disable_path_tolerance:
            # -1 erases a tolerance; 0 would inherit the controller default.
            return [
                tolerance_type(name=name, position=-1.0, velocity=-1.0, acceleration=-1.0)
                for name in JOINTS
            ]
        return [tolerance_type(name=name, position=0.15) for name in JOINTS]

    def cancel(self):
        self.closed = True
        if self.handle is not None:
            self.handle.cancel_goal_async()
