"""Offline tests: controller independence, paired episodes and raw timing."""

import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation

from dual_franka.capture.recorder import Recorder
from dual_franka.capture.sink import RecordingSink
from dual_franka.config import ROOT, Configuration
from dual_franka.control.mapping import Mapper
from dual_franka.control.panda import JOINTS, PandaCartesianController
from dual_franka.driver import KD, KP, controller_parameters
from dual_franka.session import ButtonEdges, Session


def frame(left=None, right=None, **buttons):
    def pose():
        return {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "squeeze": 0.0,
            "trigger": 0.0,
            "thumbstick": False,
            "secondary": False,
            "select": False,
        }

    left = pose() if left is None else left
    right = pose() if right is None else right
    return {
        "source": "test",
        "epoch": 0,
        "sequence": 1,
        "reference_space": "local-floor",
        "controllers": {"left": left, "right": right},
        "controller_buttons": {
            "left": {"select": buttons.get("x", False), "secondary": buttons.get("y", False)},
            "right": {"select": buttons.get("a", False), "secondary": False},
        },
        "timing": {"host_offset_ns": 0, "sync_age_ms": 1},
    }


class Model:
    lower = np.full(7, -3.0)
    upper = np.full(7, 3.0)
    velocity = np.full(7, 2.0)

    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q)[:3]
        return pose

    def solve(self, target, q, dt):
        result = np.array(q, dtype=float)
        result[:3] = target[:3, 3]
        return result


class MappingTests(unittest.TestCase):
    def engage(self, mapper, f):
        measured = np.eye(4)
        mapper.update(f, measured, 0.02)
        f["controllers"][mapper.hand]["thumbstick"] = True
        mapper.update(f, measured, 0.02)
        f["controllers"][mapper.hand]["thumbstick"] = False
        mapper.update(f, measured, 0.02)
        f["controllers"][mapper.hand]["squeeze"] = 1.0
        mapper.update(f, measured, 0.02)

    def test_each_mapper_uses_only_its_own_pose_and_clutch(self):
        mappers = {
            hand: Mapper(
                np.eye(3),
                scale=1.0,
                speed=10.0,
                angular_speed=10.0,
                radius=2.0,
                require_reset=True,
                hand=hand,
            )
            for hand in ("left", "right")
        }
        f = frame()
        for mapper in mappers.values():
            self.engage(mapper, f)
        f["controllers"]["left"]["position"][0] = 0.1
        left, _ = mappers["left"].update(f, np.eye(4), 0.02)
        right, _ = mappers["right"].update(f, np.eye(4), 0.02)
        self.assertAlmostEqual(left[0, 3], 0.1)
        self.assertAlmostEqual(right[0, 3], 0.0)
        f["controllers"]["left"]["squeeze"] = 0.0
        mappers["left"].update(f, np.eye(4), 0.02)
        mappers["right"].update(f, np.eye(4), 0.02)
        self.assertFalse(mappers["left"].engaged)
        self.assertTrue(mappers["right"].engaged)

    def test_tracking_loss_pauses_one_arm(self):
        mapper = Mapper(np.eye(3), hand="left", require_reset=True)
        f = frame()
        self.engage(mapper, f)
        f["controllers"]["left"] = None
        mapper.update(f, np.eye(4), 0.02)
        self.assertFalse(mapper.engaged)

    def test_independent_planners_and_rotation_precision(self):
        left = PandaCartesianController(model=Model(), speed=2.0)
        right = PandaCartesianController(model=Model(), speed=2.0)
        target = np.eye(4)
        target[0, 3] = 0.03
        left.update(target, np.zeros(7), np.zeros(7), True, 1.0)
        self.assertIsNone(right.segment)
        mapper = Mapper(
            np.eye(3),
            hand="left",
            scale=1.0,
            speed=10.0,
            angular_speed=10.0,
            radius=2.0,
            precision_scale=0.5,
        )
        f = frame()
        self.engage(mapper, f)
        f["controllers"]["left"]["secondary"] = True
        mapper.update(f, np.eye(4), 0.02)
        f["controllers"]["left"]["position"][0] = 0.1
        f["controllers"]["left"]["orientation"] = (
            Rotation.from_rotvec([0, 0, 0.2]).as_quat().tolist()
        )
        target, _ = mapper.update(f, np.eye(4), 0.02)
        self.assertAlmostEqual(target[0, 3], 0.05)
        self.assertAlmostEqual(Rotation.from_matrix(target[:3, :3]).magnitude(), 0.1)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        config = Configuration.load(ROOT / "config/robots.json")
        self.arms = {}
        for hand, arm_config in config.arms.items():
            mapper = SimpleNamespace(
                ready=True, basis=np.eye(3), origin=np.zeros(3), suspend=Mock()
            )
            self.arms[hand] = SimpleNamespace(
                config=arm_config,
                parameters={},
                xml="<robot/>",
                mapper=mapper,
                measured=np.eye(4),
                planner=SimpleNamespace(horizon=0.35, reset=Mock()),
                state=SimpleNamespace(q=[0.0] * 7, dq=[0.0] * 7),
                gripper=SimpleNamespace(busy=False, toggle=SimpleNamespace(closed=False)),
            )
        self.sink = RecordingSink()
        self.session = Session(
            Mock(),
            config,
            self.arms,
            self.sink,
            SimpleNamespace(metadata={}, restart=Mock()),
            self.tmp.name,
            False,
        )
        self.session.phase = "ready"
        self.session.recorder = Recorder(self.tmp.name, {"schema_version": 2}, name="test")
        self.sink.attach(self.session.recorder)
        self.session.prompts = ["insert gear"]
        self.session.reset = Mock()
        self.session.reset.tick.return_value = True
        self.now = time.monotonic()
        self.session.tick(frame(), self.now, [])

    def tearDown(self):
        self.session.close("test finished")
        self.tmp.cleanup()

    def test_a_starts_both_and_stop_disables_both_before_reset(self):
        self.session.tick(frame(a=True), self.now + 0.02, [])
        self.assertEqual(self.session.phase, "recording")
        self.session.tick(frame(), self.now + 0.04, [])
        allowed = self.session.tick(frame(a=True), self.now + 0.06, [])
        self.assertFalse(allowed)
        self.assertEqual(self.session.phase, "stopping")
        for arm in self.arms.values():
            self.assertGreaterEqual(arm.mapper.suspend.call_count, 2)
        self.session.tick(frame(), self.now + 1, [])
        self.assertEqual(self.session.phase, "resetting")
        self.session.tick(frame(), self.now + 1.02, [])
        self.assertEqual(self.session.phase, "reference")
        for arm in self.arms.values():
            self.assertFalse(arm.mapper.ready)

    def test_x_y_survive_pose_occlusion(self):
        self.session.tick(frame(a=True), self.now + 0.02, [])
        f = frame(y=True)
        f["controllers"]["left"] = None
        self.session.tick(f, self.now + 0.04, [])
        self.assertEqual(self.session.outcome, "interrupted")

    def test_lost_tracking_keeps_episode(self):
        self.session.tick(frame(a=True), self.now + 0.02, [])
        self.assertTrue(self.session.tick(None, self.now + 0.04, []))
        self.assertEqual(self.session.phase, "recording")

    def test_source_epoch_never_generates_held_button_edge(self):
        edges = ButtonEdges()
        self.assertEqual(edges.update(frame(a=True)), set())
        edges.update(frame())
        self.assertEqual(edges.update(frame(a=True)), {"A"})
        f = frame(a=True)
        f["epoch"] = 1
        self.assertEqual(edges.update(f), set())

    def test_same_episode_stores_both_arms_and_clutch_pauses(self):
        self.session.tick(frame(a=True), self.now + 0.02, [])
        receipt = time.monotonic_ns()
        for hand in self.arms:
            self.sink.record(hand + "_control", {"engaged": False}, receipt)
            self.sink.record(hand + "_robot_state", {"q": [0.0] * 7}, receipt)
        color = np.ones((2, 4, 3), np.uint8)
        depth = np.ones((2, 4), np.uint16)
        self.sink.image(
            "wrist_left",
            color,
            depth,
            {"received_ns": receipt, "depth_timing": {"source_timestamp_ms": 12}},
        )
        root = self.session.recorder.root
        self.session.recorder.end("success")
        self.session.close("done")
        episode = root / "episode_000001"
        for hand in self.arms:
            row = json.loads((episode / f"{hand}_control.jsonl").read_text())
            self.assertEqual(row["received_ns"], receipt)
            self.assertFalse(row["data"]["engaged"])
        manifest = json.loads((episode / "camera_wrist_left.jsonl").read_text())
        self.assertEqual(manifest["depth_timing"]["source_timestamp_ms"], 12)
        np.testing.assert_array_equal(np.load(episode / manifest["depth"]), depth)


class DriverTests(unittest.TestCase):
    def test_namespaces_are_isolated_and_gains_match_single_arm(self):
        left, right = controller_parameters("left"), controller_parameters("right")
        self.assertFalse(set(left) & set(right))
        gains = left["/left/joint_trajectory_controller"]["ros__parameters"]["gains"]
        self.assertEqual([gains[j]["p"] for j in JOINTS], KP)
        self.assertEqual([gains[j]["d"] for j in JOINTS], KD)

    def test_duplicate_robot_identity_rejected(self):
        data = json.loads((ROOT / "config/robots.json").read_text())
        data["arms"]["right"]["ip"] = data["arms"]["left"]["ip"]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "distinct"):
                Configuration.load(path)


if __name__ == "__main__":
    unittest.main()
