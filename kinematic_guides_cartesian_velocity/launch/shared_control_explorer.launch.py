#!/usr/bin/env python3

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
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    use_poc2 = LaunchConfiguration("use_POC2")
    launch_rviz = LaunchConfiguration("launch_rviz")


    declared_arguments = [
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
                "launch_rviz",
                default_value="false",
                description="Launch RViz?"
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
    velocity_config = PathJoinSubstitution([
        FindPackageShare("kinematic_guides_cartesian_velocity"), "config", "explorer_params.yaml"
    ])
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],
        output="both",
        remappings=[("~/robot_description", "/robot_description")]
    )

    delayed_control_node = TimerAction(
        period=1.0, 
        actions=[control_node]
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

    # Spawner for kinematic_guides_cartesian_velocity controller
    # Spawn AFTER joint_state_broadcaster and fault_controller to ensure joints and manager are ready
    kinematic_guides_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kinematic_guides_cartesian_velocity",
            "-c",
            "/controller_manager",
            "--controller-type",
            "kinematic_guides_cartesian_velocity/SharedControlVelocityController",
            "--param-file",
            velocity_config,
        ],
        output="screen",
    )

    # --- Teleoperation Node ---
    teleop_config_file = PathJoinSubstitution([
        FindPackageShare("joystick_interface"), "config", "franka_joystick_parameters.yaml"
    ])

    teleop_node =Node(
        package='joystick_interface',
        executable='joystick_input_node',
        name='joystick_input_node',
        output='screen',
        parameters=[teleop_config_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="log",
        condition=IfCondition(launch_rviz),
    )

    # Goal markers publisher (RViz visualization)
    goal_marker_node = Node(
        package="kinematic_guides_cartesian_velocity",
        executable="goal_marker_publisher",
        name="goal_marker_publisher",
        output="screen",
        parameters=[
            velocity_config,
            {"base_frame": "base_link"},
            {"ee_frame": "end_effector_link"},
            {"conf_topic": "/debug/goal_confidences"},
            {"soft_goal_topic": "/debug/soft_goal"},
            {"agnostic_goal_topic": "/debug/agnostic_goal"},
            {"raw_velocity_topic": "/debug/raw_velocity"},
            {"rate_hz": 10.0},
        ],
        condition=IfCondition(launch_rviz),
    )

    spawners_group = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner]
            )
        )


    delayed_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[kinematic_guides_spawner]
        )
    )

    rviz_start_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[rviz_node]
        )
    )

    nodes_to_start = [
            robot_state_publisher_node,
            delayed_control_node,
            spawners_group,
            delayed_robot_controller,
            rviz_start_event,
            goal_marker_node, 
            teleop_node]

    return LaunchDescription(declared_arguments + nodes_to_start)