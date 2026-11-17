"""Two isolated ROS controller managers using the installed Panda hardware driver.

Each arm keeps the proven panda_joint* names inside its own namespace. There
is no implicit controller switch, motion goal, recovery, or gripper command.
"""

import tempfile
from pathlib import Path

import yaml

from dual_franka.control.panda import JOINTS

KP = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
KD = [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]


def controller_parameters(namespace):
    types = {
        "joint_trajectory_controller": "joint_trajectory_controller/JointTrajectoryController",
        "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
        "franka_robot_state_broadcaster": "franka_robot_state_broadcaster/FrankaRobotStateBroadcaster",
    }
    return {
        f"/{namespace}/controller_manager": {
            "ros__parameters": {
                "update_rate": 1000,
                **{name: {"type": kind} for name, kind in types.items()},
            }
        },
        f"/{namespace}/joint_trajectory_controller": {
            "ros__parameters": {
                "joints": JOINTS,
                "command_interfaces": ["effort"],
                "state_interfaces": ["position", "velocity"],
                "gains": {
                    name: {"p": kp, "d": kd, "i": 0.0, "i_clamp": 1.0}
                    for name, kp, kd in zip(JOINTS, KP, KD)
                },
            }
        },
        f"/{namespace}/joint_state_broadcaster": {"ros__parameters": {"use_local_topics": False}},
        f"/{namespace}/franka_robot_state_broadcaster": {
            "ros__parameters": {"arm_id": "panda", "frequency": 100}
        },
    }


def description(ip, fake=False):
    import xacro
    from ament_index_python.packages import get_package_share_directory

    source = (
        Path(get_package_share_directory("franka_description")) / "robots/real/panda_arm.urdf.xacro"
    )
    xml = (
        xacro.process_file(
            str(source),
            mappings={
                "robot_ip": ip,
                "arm_id": "panda",
                "hand": "true",
                "use_fake_hardware": str(fake).lower(),
                "fake_sensor_commands": "false",
            },
        )
        .toxml()
        .replace("fake_components/GenericSystem", "mock_components/GenericSystem")
    )
    if fake:
        # Jazzy mock hardware reads initial_value on each state interface.
        # This older Panda xacro keeps initial_position at joint level instead.
        import xml.etree.ElementTree as ET

        tree = ET.fromstring(xml)
        for joint in tree.findall(".//ros2_control/joint"):
            initial = joint.find("param[@name='initial_position']")
            for interface in joint.findall("state_interface"):
                value = (
                    initial.text
                    if initial is not None and interface.get("name") == "position"
                    else "0.0"
                )
                ET.SubElement(interface, "param", name="initial_value").text = value
        xml = ET.tostring(tree, encoding="unicode")
    return xml


def launch_drivers(config, *, fake=False, state_only=False):
    from launch import LaunchDescription, LaunchService
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    actions = []
    with tempfile.TemporaryDirectory(prefix="dual-franka-driver-") as tmp:
        for arm in config.arms.values():
            params = Path(tmp) / f"{arm.namespace}.yaml"
            params.write_text(yaml.safe_dump(controller_parameters(arm.namespace)))
            xml = description(arm.ip, fake)
            manager = Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=arm.namespace,
                output="screen",
                parameters=[
                    str(params),
                    {"robot_description": ParameterValue(xml, value_type=str)},
                ],
                remappings=[("/robot_description", arm.topic("robot_description"))],
            )
            actions.extend(
                [
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        namespace=arm.namespace,
                        name="robot_state_publisher",
                        output="screen",
                        parameters=[
                            {
                                "robot_description": ParameterValue(xml, value_type=str),
                                "frame_prefix": f"{arm.namespace}/",
                            }
                        ],
                        remappings=[
                            ("/tf", arm.topic("tf")),
                            ("/tf_static", arm.topic("tf_static")),
                        ],
                    ),
                    manager,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=manager,
                            on_exit=[
                                EmitEvent(
                                    event=Shutdown(
                                        reason=f"{arm.hand} driver exited; stopping both drivers"
                                    )
                                )
                            ],
                        )
                    ),
                ]
            )
            controllers = ["joint_state_broadcaster"]
            if not state_only:
                controllers.append("joint_trajectory_controller")
            if not fake:
                controllers.append("franka_robot_state_broadcaster")
            for controller in controllers:
                actions.append(
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        namespace=arm.namespace,
                        output="screen",
                        arguments=[
                            controller,
                            "--controller-manager",
                            arm.topic("controller_manager"),
                            "--controller-manager-timeout",
                            "30",
                        ],
                    )
                )
            if fake:
                actions.append(
                    Node(
                        package="franka_gripper",
                        executable="fake_gripper_state_publisher.py",
                        namespace=arm.namespace,
                        name="panda_gripper",
                        parameters=[
                            {"joint_names": ["panda_finger_joint1", "panda_finger_joint2"]}
                        ],
                    )
                )
            else:
                actions.append(
                    Node(
                        package="franka_gripper",
                        executable="franka_gripper_node",
                        namespace=arm.namespace,
                        name="panda_gripper",
                        output="screen",
                        parameters=[
                            {
                                "robot_ip": arm.ip,
                                "joint_names": ["panda_finger_joint1", "panda_finger_joint2"],
                                "state_publish_rate": 50,
                                "feedback_publish_rate": 30,
                                "default_speed": 0.1,
                            }
                        ],
                    )
                )
        service = LaunchService()
        service.include_launch_description(LaunchDescription(actions))
        return service.run()
