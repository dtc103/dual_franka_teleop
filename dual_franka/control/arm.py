"""One arm's ROS state, relative Quest mapping, IK and trajectory publication."""

import copy
import time

import numpy as np
from scipy.spatial.transform import Rotation

from dual_franka.control.gripper import Gripper
from dual_franka.control.mapping import Mapper
from dual_franka.control.panda import JOINTS, PandaCartesianController


def wait_for(node, future, timeout=8):
    import rclpy

    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise RuntimeError("ROS request timed out")
        rclpy.spin_once(node, timeout_sec=0.02)
    return future.result()


class Arm:
    def __init__(self, node, config, settings, recorder, execute):
        from controller_manager_msgs.srv import ListControllers
        from franka_msgs.msg import FrankaState
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.node, self.config, self.settings = node, config, settings
        self.recorder, self.execute = recorder, execute
        self.planner = None
        self.publisher = None
        self.state = None
        self.width = None
        self.state_at = self.width_at = 0.0
        self.measured = None
        self.target = None
        self.xml = None
        self.parameters = {}
        self.manager = node.create_client(
            ListControllers, config.topic("controller_manager/list_controllers")
        )
        self.manager_future = None
        self.manager_at = self.manager_requested = 0.0
        self.active = False
        self.subscriptions = [
            node.create_subscription(
                FrankaState,
                config.topic("franka_robot_state_broadcaster/robot_state"),
                self._state,
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                JointState,
                config.topic("panda_gripper/joint_states"),
                self._gripper_state,
                qos_profile_sensor_data,
            ),
        ]
        basis = Rotation.from_euler("z", config.yaw_deg, degrees=True).as_matrix() @ np.array(
            [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        )
        self.mapper = Mapper(
            basis,
            settings["scale"],
            settings["max_speed"],
            settings["max_angular_speed"],
            settings["workspace_radius"],
            settings["smoothing_seconds"],
            require_reset=True,
            frame_alignment=settings["frame_alignment"],
            precision_scale=settings["precision_scale"],
            hand=config.hand,
        )
        self.gripper = Gripper(node, config, settings, self.record_message, execute)

    def record_message(self, stream, message):
        from rosidl_runtime_py.convert import message_to_ordereddict

        self.recorder.record(f"{self.config.hand}_{stream}", message_to_ordereddict(message))

    def _state(self, message):
        self.state, self.state_at = message, time.monotonic()
        self.record_message("robot_state", message)

    def _gripper_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in ("panda_finger_joint1", "panda_finger_joint2")):
            self.width = positions["panda_finger_joint1"] + positions["panda_finger_joint2"]
            self.width_at = time.monotonic()
        self.record_message("gripper_state", message)

    def initialize(self):
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from rclpy.parameter_client import AsyncParameterClient
        from trajectory_msgs.msg import JointTrajectory

        description = AsyncParameterClient(self.node, self.config.topic("robot_state_publisher"))
        if not description.wait_for_services(timeout_sec=8):
            raise RuntimeError(f"Start the {self.config.hand} robot driver first")
        self.xml = (
            wait_for(self.node, description.get_parameters(["robot_description"]))
            .values[0]
            .string_value
        )
        s = self.settings
        self.planner = PandaCartesianController(
            self.xml,
            horizon=s["trajectory_horizon"],
            speed=s["joint_speed"],
            acceleration=s["joint_acceleration"],
            jerk=s["joint_jerk"],
        )
        controller = AsyncParameterClient(
            self.node, self.config.topic("joint_trajectory_controller")
        )
        if not controller.wait_for_services(timeout_sec=8) or not self.manager.wait_for_service(
            timeout_sec=8
        ):
            raise RuntimeError(f"{self.config.hand} joint trajectory controller unavailable")
        names = wait_for(self.node, controller.list_parameters()).result.names
        values = wait_for(self.node, controller.get_parameters(names)).values
        from rosidl_runtime_py.convert import message_to_ordereddict

        self.parameters = {
            name: message_to_ordereddict(value) for name, value in zip(names, values)
        }
        joints = self.parameters["joints"]["string_array_value"]
        if list(joints) != JOINTS:
            raise RuntimeError(f"{self.config.hand} controller joint order differs from its model")
        response = wait_for(self.node, self.manager.call_async(ListControllers.Request()))
        self.active = any(
            c.name == "joint_trajectory_controller" and c.state == "active"
            for c in response.controller
        )
        self.manager_at = self.manager_requested = time.monotonic()
        if not self.active:
            raise RuntimeError(f"{self.config.hand} joint trajectory controller is inactive")
        deadline = time.monotonic() + 8
        while self.state is None or self.width is None:
            if time.monotonic() > deadline:
                missing = []
                if self.state is None:
                    missing.append(self.config.topic("franka_robot_state_broadcaster/robot_state"))
                if self.width is None:
                    missing.append(self.config.topic("panda_gripper/joint_states"))
                raise RuntimeError("No messages received on: " + ", ".join(missing))
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.gripper.toggle.closed = self.width < 0.04
        self.refresh_pose()
        command_topic = self.config.topic("joint_trajectory_controller/joint_trajectory")
        if self.execute:
            if self.node.count_publishers(command_topic):
                raise RuntimeError(f"Another publisher is controlling {self.config.hand}")
            self.publisher = self.node.create_publisher(JointTrajectory, command_topic, 1)
        self.health(time.monotonic())

    def refresh_pose(self):
        # Same Panda TCP FK used by the single-arm pose broadcaster, from measured q.
        self.measured = self.planner.model.fk(self.state.q)
        self.target = self.measured.copy() if self.target is None else self.target

    def health(self, now):
        from controller_manager_msgs.srv import ListControllers

        if self.manager_future is not None and self.manager_future.done():
            response = self.manager_future.result()
            self.active = any(
                c.name == "joint_trajectory_controller" and c.state == "active"
                for c in response.controller
            )
            self.manager_at, self.manager_future = now, None
        if now - self.manager_requested > 0.5 and self.manager_future is None:
            self.manager_future = self.manager.call_async(ListControllers.Request())
            self.manager_requested = now
        if not self.active or now - self.manager_at > 2:
            raise RuntimeError(f"{self.config.hand} controller inactive or unavailable")
        if (
            self.execute
            and self.node.count_publishers(
                self.config.topic("joint_trajectory_controller/joint_trajectory")
            )
            > 1
        ):
            raise RuntimeError(f"Another publisher is controlling {self.config.hand}")
        if now - min(self.state_at, self.width_at) > self.settings["stale_seconds"]:
            raise RuntimeError(f"{self.config.hand} robot/gripper state is stale")
        errors = [
            name
            for name in self.state.current_errors.get_fields_and_field_types()
            if getattr(self.state.current_errors, name)
        ]
        if errors or self.state.robot_mode not in (1, 2):
            mode = {
                0: "Other",
                3: "Guiding",
                4: "Reflex",
                5: "User stopped",
                6: "Error recovery",
            }.get(self.state.robot_mode, str(self.state.robot_mode))
            previous = [
                name
                for name in self.state.last_motion_errors.get_fields_and_field_types()
                if getattr(self.state.last_motion_errors, name)
            ]
            raise RuntimeError(
                f"{self.config.hand.capitalize()} robot is in {mode}. "
                f"Current errors: {errors or 'none'}; last motion errors: {previous or 'none'}. "
                "Restore the robot to a ready state in Franka Desk, then click Reconnect robots."
            )
        self.gripper.tick()
        self.refresh_pose()

    def send(self, segment):
        if segment is None or self.publisher is None:
            return
        from rclpy.duration import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        message = JointTrajectory()
        message.joint_names = JOINTS
        for seconds, q, dq, ddq in self.planner.trajectory_points(segment):
            point = JointTrajectoryPoint()
            point.positions, point.velocities, point.accelerations = (
                q.tolist(),
                dq.tolist(),
                ddq.tolist(),
            )
            point.time_from_start = Duration(seconds=seconds).to_msg()
            message.points.append(point)
        self.publisher.publish(message)
        self.record_message("sent_command", message)

    def step(self, frame, dt, now, allow_input):
        # Right B is a shared precision clutch, leaving left Y available to interrupt.
        physical_hand = None if frame is None else frame["controllers"][self.config.hand]
        adapted = copy.deepcopy(frame)
        hand = None if adapted is None else adapted["controllers"][self.config.hand]
        if hand is not None:
            right = frame["controllers"]["right"]
            hand["secondary"] = bool(right and right.get("secondary", False))
            # Share the reference button, but retain each controller's own pose.
            # Both poses must be available before either reference can be reset.
            both_tracked = all(
                frame["controllers"].get(side) is not None for side in ("left", "right")
            )
            hand["thumbstick"] = bool(both_tracked and right.get("thumbstick", False))
            if not allow_input:
                hand["squeeze"] = 0.0
                hand["trigger"] = 1.0
        self.target, clicked = self.mapper.update(adapted, self.measured, dt)
        if not self.mapper.ready and frame is not None:
            missing = [side for side, pose in frame["controllers"].items() if pose is None]
            if missing:
                self.mapper.status = (
                    f"Reference blocked: {', '.join(missing)} tracking unavailable; "
                    "both controllers must be tracked before clicking RIGHT thumbstick"
                )
        if not allow_input:
            self.mapper.engaged = False
            clicked = False
        reference = self.planner.commanded_pose(now, self.measured)
        position_error = float(np.linalg.norm(reference[:3, 3] - self.measured[:3, 3]))
        angle_error = float(
            Rotation.from_matrix(reference[:3, :3] @ self.measured[:3, :3].T).magnitude()
        )
        if self.execute and (position_error > 0.08 or angle_error > 0.5):
            self.mapper.suspend(self.measured)
            self.target = self.mapper.target.copy()
            clicked = False
            self.mapper.status = "Execution tracking error: release grip and re-engage"
        segment = None
        if self.mapper.ready or self.planner.moving:
            try:
                segment = self.planner.update(
                    self.target, self.state.q, self.state.dq, self.mapper.engaged, now
                )
            except ValueError as error:
                self.mapper.suspend(self.measured)
                self.mapper.status = f"{error}; release grip and re-engage"
                clicked = False
                segment = self.planner.update(
                    self.measured, self.state.q, self.state.dq, False, now
                )
        self.gripper.tick(clicked)
        sample = {
            "command_time_ns": time.monotonic_ns(),
            "target": self.target.tolist(),
            "measured": self.measured.tolist(),
            "engaged": self.mapper.engaged,
            "clutch_pressed": bool(physical_hand and physical_hand["squeeze"] > 0.8),
            "tracking_available": hand is not None,
            "input_allowed": allow_input,
            "status": self.mapper.status,
            "scale": self.mapper.scale,
            "rotation_scale": self.mapper.rotation_scale,
            "gripper_closed": self.gripper.toggle.closed,
            "gripper_measured_width_m": self.width,
            "q": list(self.state.q),
            "dq": list(self.state.dq),
            "quest_sequence": None if frame is None else frame["sequence"],
            "tracking_position_m": position_error,
            "tracking_rotation_rad": angle_error,
        }
        return segment, sample

    def stop(self, now):
        self.mapper.suspend(self.measured)
        self.gripper.cancel()
        if self.planner is None or self.state is None or self.measured is None:
            return
        if now - self.state_at > self.settings["stale_seconds"] or self.state.robot_mode not in (
            1,
            2,
        ):
            return  # Previously published finite segment ends at rest.
        if any(
            getattr(self.state.current_errors, key)
            for key in self.state.current_errors.get_fields_and_field_types()
        ):
            return
        self.send(self.planner.update(self.measured, self.state.q, self.state.dq, False, now))
