"""Offline tests with real ROS message types, never robot drivers."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import test_dual

from dual_franka.config import ROOT, Configuration
from dual_franka.control.arm import Arm
from dual_franka.control.mapping import Mapper
from dual_franka.control.panda import PandaCartesianController


class ArmAdapterTests(unittest.TestCase):
    def arm(self, hand):
        arm = Arm.__new__(Arm)
        arm.config = Configuration.load(ROOT / "config/robots.json").arms[hand]
        arm.mapper = Mapper(
            np.eye(3),
            hand=hand,
            require_reset=True,
            scale=1.0,
            speed=10.0,
            angular_speed=10.0,
            radius=2.0,
        )
        arm.planner = PandaCartesianController(model=test_dual.Model(), speed=2.0)
        arm.state = SimpleNamespace(q=[0.0] * 7, dq=[0.0] * 7)
        arm.measured = arm.target = np.eye(4)
        arm.width = 0.08
        arm.gripper = SimpleNamespace(tick=Mock(), toggle=SimpleNamespace(closed=False))
        arm.execute = False
        arm.publisher = Mock()
        arm.record_message = Mock()
        return arm

    def test_reflex_is_reported_even_with_empty_current_errors(self):
        import time
        from franka_msgs.msg import FrankaState

        arm = self.arm("right")
        arm.settings = Configuration.load(ROOT / "config/robots.json").control
        arm.manager_future = None
        arm.manager_at = arm.manager_requested = arm.state_at = arm.width_at = time.monotonic()
        arm.active = True
        arm.state = FrankaState()
        arm.state.robot_mode = FrankaState.ROBOT_MODE_REFLEX
        arm.state.last_motion_errors.joint_velocity_violation = True
        with self.assertRaisesRegex(RuntimeError, "right|Right") as error:
            arm.health(time.monotonic())
        self.assertIn("Reflex", str(error.exception))
        self.assertIn("joint_velocity_violation", str(error.exception))
        self.assertIn("Reconnect robots", str(error.exception))

    def test_right_thumbstick_sets_both_references(self):
        arms = {hand: self.arm(hand) for hand in ("left", "right")}
        arms["left"].measured = np.eye(4)
        arms["left"].measured[0, 3] = 0.12
        f = test_dual.frame()
        f["controllers"]["left"]["position"] = [0.3, 0.0, 0.0]
        for arm in arms.values():
            arm.step(f, 0.02, 1.0, False)
        f["controllers"]["right"]["thumbstick"] = True
        for arm in arms.values():
            arm.step(f, 0.02, 1.02, False)
            self.assertTrue(arm.mapper.ready)
            np.testing.assert_array_equal(arm.mapper.target, arm.measured)
        self.assertFalse(f["controllers"]["left"]["thumbstick"])

    def test_left_thumbstick_does_not_set_references(self):
        arms = {hand: self.arm(hand) for hand in ("left", "right")}
        f = test_dual.frame()
        for arm in arms.values():
            arm.step(f, 0.02, 1.0, False)
        f["controllers"]["left"]["thumbstick"] = True
        for arm in arms.values():
            arm.step(f, 0.02, 1.02, False)
            self.assertFalse(arm.mapper.ready)

    def test_reference_requires_both_controller_poses(self):
        arm = self.arm("right")
        f = test_dual.frame()
        arm.step(f, 0.02, 1.0, False)
        f["controllers"]["left"] = None
        f["controllers"]["right"]["thumbstick"] = True
        arm.step(f, 0.02, 1.02, False)
        self.assertFalse(arm.mapper.ready)
        self.assertIn("left tracking unavailable", arm.mapper.status)

    def test_shared_b_scales_both_arms_without_pose_cross_talk(self):
        arms = {hand: self.arm(hand) for hand in ("left", "right")}
        f = test_dual.frame()
        for arm in arms.values():
            test_dual.MappingTests().engage(arm.mapper, f)
        f["controllers"]["right"]["secondary"] = True
        for arm in arms.values():
            arm.step(f, 0.02, 1.0, True)
            self.assertEqual(arm.mapper.scale, 0.5)
            self.assertEqual(arm.mapper.rotation_scale, 0.5)
        self.assertFalse(f["controllers"]["left"]["secondary"])

    def test_stop_edge_removes_grip_and_gripper_for_each_arm(self):
        arm = self.arm("left")
        f = test_dual.frame()
        test_dual.MappingTests().engage(arm.mapper, f)
        f["controllers"]["left"]["trigger"] = 1.0
        _, sample = arm.step(f, 0.02, 1.0, False)
        self.assertFalse(sample["engaged"])
        self.assertFalse(sample["input_allowed"])
        arm.gripper.tick.assert_called_once_with(False)

    def test_published_trajectory_is_independent_and_has_eight_timed_points(self):
        left, right = self.arm("left"), self.arm("right")
        segment = left.planner.update(np.eye(4), np.zeros(7), np.zeros(7), True, 1.0)
        left.send(segment)
        message = left.publisher.publish.call_args.args[0]
        self.assertEqual(len(message.points), 8)
        self.assertEqual(message.header.stamp.sec, 0)
        self.assertEqual(message.points[0].time_from_start.nanosec, 0)
        self.assertGreater(message.points[-1].time_from_start.nanosec, 0)
        right.publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()


class BrowserDisconnectTests(unittest.TestCase):
    def test_browser_cancel_does_not_escape_request_handler(self):
        from dual_franka.quest.reader import Handler

        handler = Handler.__new__(Handler)
        for error in (BrokenPipeError(), ConnectionResetError()):
            with patch.object(handler, "_send_reply", side_effect=error):
                handler.reply(200, b"page")

    def test_unrelated_server_error_still_propagates(self):
        from dual_franka.quest.reader import Handler

        handler = Handler.__new__(Handler)
        with patch.object(handler, "_send_reply", side_effect=ValueError("bad response")):
            with self.assertRaises(ValueError):
                handler.reply(200, b"page")
