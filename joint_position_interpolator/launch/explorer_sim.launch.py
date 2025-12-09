import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    pkg_ros2_control_explorer = FindPackageShare("ros2_control_explorer")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_interpolator = FindPackageShare("joint_position_interpolator")

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start RViz2 automatically with this launch file.",
    )

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="If true, use simulated clock",
    )

    # --------------------------------------------------------------------------
    # 2. File Paths & Substitutions
    # --------------------------------------------------------------------------
    world_path = PathJoinSubstitution(
        [pkg_ros2_control_explorer, "description", "worlds", "empty_world.world"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_ros2_control_explorer, "description", "urdf", "explorer.urdf.xacro"]
            ),
            " ",
            "use_ignition:=true",
            " ",
            "use_POC2:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [pkg_ros2_control_explorer, "description", "rviz", "view_robot.rviz"]
    )

    bridge_config = PathJoinSubstitution(
        [pkg_ros2_control_explorer, "config", "bridge.yaml"]
    )

    controller_config = PathJoinSubstitution(
        [pkg_interpolator, "config", "explorer.yaml"]
    )

    # --------------------------------------------------------------------------
    # 3. Nodes
    # --------------------------------------------------------------------------
    ignition_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    ignition_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": "-g -v4"}.items(),
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "explorer",
            "-allow_renaming", "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )

    joint_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_position_interpolator",
            "-c", "/controller_manager",
            "--param-file", controller_config,
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    start_gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            ["config_file:=", bridge_config],
        ],
        output="screen",
    )

    start_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera", "depth_camera", "rgbd_camera/image", "rgbd_camera/depth_image"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # 4. Event Handlers
    # --------------------------------------------------------------------------
    spawn_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_pos_controller_spawner],
        )
    )

    # --------------------------------------------------------------------------
    # 5. Launch Description
    # --------------------------------------------------------------------------

    nodes_and_events = [
        declare_gui,
        declare_sim_time,        
        ignition_server,
        ignition_client,
        node_robot_state_publisher,
        gz_spawn_entity,
        start_gazebo_ros_bridge,
        start_image_bridge,
        delayed_rviz,
        spawn_broadcaster_event,
        spawn_controller_event,
    ]

    return LaunchDescription(nodes_and_events)