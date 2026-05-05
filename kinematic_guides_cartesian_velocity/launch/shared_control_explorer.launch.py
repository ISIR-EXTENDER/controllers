import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simulation = LaunchConfiguration("use_simulation")
    spacenav = LaunchConfiguration("spacenav")
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    use_poc2 = LaunchConfiguration("use_POC2")
    use_joystick_interface = LaunchConfiguration("use_joystick_interface")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "use_actuator_interface",
            default_value="true",
            description="Use VESCInterface to control the robot. Set to false for simulation",
        ),
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="CAN Port for VESC Communication",
        ),
        DeclareLaunchArgument(
            "host_id",
            default_value="45",
            description="Host CAN ID for VESC Communication",
        ),
        DeclareLaunchArgument(
            "use_POC2", default_value="true", description="Use POC2 urdf"
        ),
        DeclareLaunchArgument(
            "spacenav",
            default_value="True",
            description="If the spacenav 3D mouse is used",
        ),
        DeclareLaunchArgument(
            "use_joystick_interface",
            default_value="false",
            description="Start joystick_interface node (disable when using tablette UI).",
        ),
    ]

    # --------------------------------------------------------------------------
    # 2. File Paths & Substitutions
    # --------------------------------------------------------------------------
    pkg_share = FindPackageShare("explorer_description")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "explorer.urdf.xacro"]),
            " ",
            "use_ignition:=",
            use_simulation,
            "use_actuator_interface:=",
            use_actuator_interface,
            " can_port:=",
            can_port,
            " host_id:=",
            host_id,
            " use_POC2:=",
            use_poc2,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    # Config Files
    velocity_config = PathJoinSubstitution(
        [FindPackageShare("kinematic_guides_cartesian_velocity"), "config", "explorer_params.yaml"]
    )

    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("explorer_bringup"), "/launch/simulation_base.launch.py"]
        ),
        launch_arguments={
            "use_POC2": use_poc2,
            "gui": gui,
            "use_sim_time": use_sim_time,
            "rviz_delay": "0.0",
            "extra_controllers_config": velocity_config,
            "use_custom_controllers": "true",
        }.items(),
        condition=IfCondition(use_simulation),
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("explorer_bringup"), "/launch/hardware_base.launch.py"]
        ),
        launch_arguments={
            "gui": gui,
            "use_sim_time": use_sim_time,
            "use_actuator_interface": "True",
            "can_port": can_port,
            "host_id": host_id,
            "use_POC2": use_poc2,
            "rviz_delay": "5.0",
            "extra_controllers_config": velocity_config
        }.items(),
        condition=UnlessCondition(use_simulation),
    )

    spawner_qontrol = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
    )

    spawner_teleop_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kinematic_guides_cartesian_velocity",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # --- Teleoperation Node ---
    teleop_config_file = PathJoinSubstitution(
        [
            FindPackageShare("joystick_interface"),
            "config",
            "explorer_joystick_parameters.yaml",
        ]
    )

    teleop_node = Node(
        package="joystick_interface",
        executable="joystick_input_node",
        name="joystick_input_node",
        output="screen",
        parameters=[teleop_config_file],
        condition=IfCondition(use_joystick_interface),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(use_joystick_interface),
    )

    # --------------------------------------------------------------------------
    # 4. Event Handlers
    # --------------------------------------------------------------------------
    # delayed_spawner_qontrol = TimerAction(
    #     period=10.0,
    #     actions=[spawner_qontrol]
    # )
    start_cartesian_teleop_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_qontrol, on_exit=[spawner_teleop_controller]
        )
    )

    # --------------------------------------------------------------------------
    # 5. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_simulation,
        robot_hardware,
        teleop_node,
        spawner_qontrol,
        start_cartesian_teleop_event,
        joy_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
