import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    spacenav = LaunchConfiguration('spacenav')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    use_poc2 = LaunchConfiguration("use_POC2")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui", 
            default_value="true", 
            description="Start RViz2 automatically with this launch file."
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            "use_actuator_interface", 
            default_value="true", 
            description="Use VESCInterface to control the robot. Set to false for simulation"
        ),
        DeclareLaunchArgument(
            "can_port", 
            default_value="vxcan1", 
            description="CAN Port for VESC Communication"
        ),
        DeclareLaunchArgument(
            "host_id", 
            default_value="45", 
            description="Host CAN ID for VESC Communication"
        ),
        DeclareLaunchArgument(
            "use_POC2", 
            default_value="true", 
            description="Use POC2 urdf"
        ),
        DeclareLaunchArgument(
            'spacenav', 
            default_value='True', 
            description='If the spacenav 3D mouse is used'
        )
    ]

    # --------------------------------------------------------------------------
    # 2. File Paths & Substitutions
    # --------------------------------------------------------------------------
    pkg_share = FindPackageShare("ros2_control_explorer")
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "description/urdf", "explorer.urdf.xacro"]), " ",
        "use_ignition:=false ",
        "use_actuator_interface:=", use_actuator_interface,
        " can_port:=", can_port,
        " host_id:=", host_id,
        " use_POC2:=", use_poc2,
    ])
    robot_description = {"robot_description": robot_description_content}
    # Config Files
    robot_controllers = PathJoinSubstitution([pkg_share, "config", "explorer_controller.yaml"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "description/rviz", "view_robot.rviz"])
    spacenav_config = PathJoinSubstitution([pkg_share, "config", "spacenav_settings.yaml"])
   
    velocity_config = PathJoinSubstitution([
        FindPackageShare("cartesian_velocity"), "config", "explorer_params.yaml"
    ])
    # --------------------------------------------------------------------------
    # 3. Standalone ros2_control Node (Replaces Gazebo)
    # --------------------------------------------------------------------------
    # This node runs the hardware interfaces and the controller manager
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    delayed_control_node = TimerAction(
        period=1.0, 
        actions=[control_node]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="log",
    )
    joint_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_velocity_teleop_controller",
            "-t", "cartesian_velocity/CartesianVelocityTeleopController",
            "--param-file", velocity_config

        ],
        output="screen",
    )

    gui_control_node = Node(
        package='rqt_armcontrol',
        executable='rqt_armcontrol',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Teleoperation Node ---
    teleop_config_file = PathJoinSubstitution([
        FindPackageShare("joystick_interface"), "config", "explorer_joystick_parameters.yaml"
    ])

    teleop_node =Node(
        package='joystick_interface',
        executable='joystick_input_node',
        name='joystick_input_node',
        output='screen',
        parameters=[teleop_config_file],
    )

    # --------------------------------------------------------------------------
    # 4. Event Handlers
    # --------------------------------------------------------------------------
    spawners_group = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner]
        )
    )


    delayed_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_pos_controller_spawner]
        )
    )


    rviz_start_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[rviz_node]
        )
    )

    # --------------------------------------------------------------------------
    # 5. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        node_robot_state_publisher,
        delayed_control_node,
        gui_control_node,
        spawners_group,
        delayed_robot_controller,
        rviz_start_event, 
        teleop_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)