#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_and_apply_prefix(yaml_path, prefix):
    """
    Load YAML file and substitute ${prefix} placeholders.
    Save the resolved YAML to a temporary file and return the path.
    """
    with open(yaml_path) as f:
        text = f.read()
    # Replace ${prefix} placeholders in the text
    text = text.replace("${prefix}", prefix)
    data = yaml.safe_load(text)

    # Save the resolved YAML to a temporary file
    dir_name = os.path.dirname(os.path.abspath(yaml_path))
    resolved_name = f"{prefix}ros2_controllers_resolved.yaml"
    resolved_path = os.path.join(dir_name, resolved_name)
    try:
        with open(resolved_path, "w") as out:
            yaml.dump(data, out, default_flow_style=False)
        print(f"[INFO] Saved resolved YAML to: {resolved_path}")
    except Exception as e:
        print(f"[WARN] Could not save resolved YAML: {e}")

    return resolved_path


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    dof = LaunchConfiguration("dof")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    prefix = LaunchConfiguration("prefix")
    gripper = LaunchConfiguration("gripper")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
    controllers_file = LaunchConfiguration("controllers_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    description_package = LaunchConfiguration("description_package")
    use_joystick_interface = LaunchConfiguration("use_joystick_interface")

    # Resolve YAML path and load controllers with prefix substitution
    try:
        cartesian_velocity_share = get_package_share_directory("cartesian_velocity")
        yaml_path_str = os.path.join(
            cartesian_velocity_share,
            "config",
            "kinova_params.yaml",
        )
        # Load YAML with empty prefix and save resolved YAML
        controllers_yaml_path = load_and_apply_prefix(yaml_path_str, "")
    except Exception as e:
        print(f"[WARN] Could not load controllers YAML: {e}")
        controllers_yaml_path = None

    # Generate robot description from xacro
    urdf_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", "gen3.xacro"]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "arm",
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "gripper:=",
            gripper,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "gripper_max_velocity:=",
            gripper_max_velocity,
            " ",
            "gripper_max_force:=",
            gripper_max_force,
            " ",
            "gripper_joint_name:=",
            gripper_joint_name,
            " ",
        ]
    )
    robot_description = {"robot_description": urdf_cmd}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ros2_control node - it will load the YAML config file specified in parameters
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml_path, robot_description],
        output="both",
    )

    # Spawner for joint_state_broadcaster
    # Use --activate flag to ensure it's fully activated before returning
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--activate",
        ],
        output="screen",
    )

    # Spawner for robotiq_gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", gripper, "' != ''"])),
        output="screen",
    )

    # Spawner for fault_controller
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Spawner for cartesian_velocity_teleop_controller
    # NOTE: This controller internally manages tcp/twist.* interfaces via GenericComponent
    #       So we don't need a separate twist_controller spawner
    # IMPORTANT: Spawned AFTER joint_state_broadcaster to ensure joint states are available
    cartesian_teleop_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_velocity_teleop_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Joystick
    joystick_params_yaml = PathJoinSubstitution(
        [
            FindPackageShare("joystick_interface"),
            "config",
            "kinova_joystick_parameters.yaml",
        ]
    )
    joystick_node = Node(
        package="joystick_interface",
        executable="joystick_input_node",
        name="joystick_input",
        output="screen",
        parameters=[joystick_params_yaml],
        condition=IfCondition(use_joystick_interface),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(use_joystick_interface),
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "rviz",
            "view_robot.rviz",
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="log",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="gen3",
                description="Type/series of robot.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="dont-care",
                description="IP address by which the robot can be reached.",
            ),
            DeclareLaunchArgument(
                "dof",
                default_value="7",
                description="DoF of robot.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Start robot with fake hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Enable fake command interfaces for sensors used for simple simulations.",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Prefix of the joint names, useful for multi-robot setup.",
            ),
            DeclareLaunchArgument(
                "gripper",
                default_value="robotiq_2f_85",
                description='Name of the gripper attached to the arm ("" for none)',
            ),
            DeclareLaunchArgument(
                "use_internal_bus_gripper_comm",
                default_value="true",
                description="Use internal bus for gripper communication?",
            ),
            DeclareLaunchArgument(
                "gripper_max_velocity",
                default_value="100.0",
                description="Max velocity for gripper commands",
            ),
            DeclareLaunchArgument(
                "gripper_max_force",
                default_value="100.0",
                description="Max force for gripper commands",
            ),
            DeclareLaunchArgument(
                "gripper_joint_name",
                default_value="robotiq_85_left_knuckle_joint",
                description="Name of the gripper joint",
            ),
            DeclareLaunchArgument(
                "description_package",
                default_value="kortex_description",
                description="Robot description package.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="kinova_params.yaml",
                description="Controllers YAML file.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz?",
            ),
            DeclareLaunchArgument(
                "use_joystick_interface",
                default_value="false",
                description="Start joystick_interface node (disable when using tablette UI).",
            ),
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            # Spawn gripper and fault controllers after joint_state_broadcaster starts
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[gripper_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper_controller_spawner,
                    on_exit=[fault_controller_spawner],
                )
            ),
            # Spawn cartesian_teleop controller after fault_controller
            RegisterEventHandler(
                OnProcessExit(
                    target_action=fault_controller_spawner,
                    on_exit=[cartesian_teleop_spawner],
                )
            ),
            joystick_node,
            joy_node,
            rviz_node,
        ]
    )
