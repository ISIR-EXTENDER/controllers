from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import tempfile
import yaml


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")
    kinova_controller_params = LaunchConfiguration("kinova_controller_params")
    use_joystick_interface = LaunchConfiguration("use_joystick_interface")

    # description_package and description_file kept flexible but default to kortex/gen3
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # Gripper configuration (same semantics as kortex_bringup gen3.launch.py)
    gripper = LaunchConfiguration("gripper")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")

    # Robot_description from kortex_description/robots/*.xacro ---
    urdf_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "robots",
                    description_file,
                ]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "arm",
            " ",
            "arm:=",
            "gen3",
            " ",
            "dof:=",
            "7",
            " ",
            "prefix:=",
            "",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            # Pass gripper argument through to the gen3.xacro, so the attached gripper is included
            "gripper:=",
            gripper,
            " ",
            "parent:=",
            "tool_frame",
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
        ]
    )
    robot_description = {"robot_description": urdf_cmd}

    # Controllers YAML
    controllers_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_velocity"),
            "config",
            "kinova_params.yaml",
        ]
    )

    def generate_controller_params(context, *args, **kwargs):
        resolved_yaml_path = context.perform_substitution(controllers_yaml_path)
        urdf_xml = context.perform_substitution(urdf_cmd)

        with open(resolved_yaml_path, "r", encoding="utf-8") as file:
            params_data = yaml.safe_load(file) or {}

        controller_params = params_data.setdefault("cartesian_velocity_teleop_controller", {})
        ros_params = controller_params.setdefault("ros__parameters", {})
        ros_params["robot_description"] = urdf_xml

        fd, temp_path = tempfile.mkstemp(prefix="kinova_params_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as temp_file:
            yaml.safe_dump(params_data, temp_file, default_flow_style=False)

        return [SetLaunchConfiguration("kinova_controller_params", temp_path)]

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Ros2 control node --- loads controllers from controllers_yaml ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml_path,
        ],
        output="both",
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_gripper_controller",
            "-c",
            "/controller_manager",
        ],
        # only spawn if gripper argument is not empty string
        condition=IfCondition(PythonExpression(["'", gripper, "' != ''"])),
        output="screen",
    )

    cartesian_teleop_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_velocity_teleop_controller",
            "-c",
            "/controller_manager",
            "--param-file",
            kinova_controller_params,
        ],
        output="screen",
    )

    # Joystick config YAML
    joystick_params_yaml = PathJoinSubstitution(
        [
            FindPackageShare("joystick_interface"),
            "config",
            "kinova_joystick_parameters.yaml",
        ]
    )

    # Joystick Interface Node
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

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
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
                "robot_ip",
                default_value="dont-care",
                description="Kinova Gen3 IP (ignored if fake hardware).",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use ros2_control fake hardware.",
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Enable fake sensor commands (only with fake hardware).",
            ),
            DeclareLaunchArgument(
                "description_package",
                default_value="kortex_description",
                description="Robot description package.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="gen3.xacro",
                description="Xacro file in kortex_description/robots/.",
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
                default_value="finger_joint",
                description="Name of the gripper joint",
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
            DeclareLaunchArgument(
                "kinova_controller_params",
                default_value=controllers_yaml_path,
                description="Override controller params file (auto-generated by launch).",
            ),
            OpaqueFunction(function=generate_controller_params),
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            gripper_controller_spawner,
            cartesian_teleop_spawner,
            joystick_node,
            joy_node,
            rviz_node,
        ]
    )
