from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

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
        ), 
        DeclareLaunchArgument(
            "use_fake_hardware", 
            default_value="false", 
            description="Use fake hardware interface"
        ),
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
        " use_fake_hardware:=", use_fake_hardware
    ])
    robot_description = {"robot_description": robot_description_content}

    # Config Files
    robot_controllers = PathJoinSubstitution([pkg_share, "config", "explorer_controller.yaml"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "description/rviz", "view_robot.rviz"])
    spacenav_config = PathJoinSubstitution([pkg_share, "config", "spacenav_settings.yaml"])
    
    interpolator_config = PathJoinSubstitution([
        FindPackageShare("joint_position_interpolator"), "config", "explorer.yaml"
    ])

    # --------------------------------------------------------------------------
    # 3. Nodes
    # --------------------------------------------------------------------------
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
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_position_interpolator",
            "-c", "/controller_manager",
            "--param-file", interpolator_config
        ],
        output='screen'
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

    spacenav_node = Node(
        package='ros2_control_explorer',
        executable='spacenav',
        parameters=[
            spacenav_config,
            {'static_rot_deadband': 0.5},
            {'static_trans_deadband': 0.5}
        ],
        condition=IfCondition(spacenav),
    )

    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        parameters=[
            {'static_rot_deadband': 0.5},
            {'static_trans_deadband': 0.5}
        ],
        condition=IfCondition(spacenav),
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
        spacenav_node,
        spacenav_driver_node,
        spawners_group,
        delayed_robot_controller,
        rviz_start_event
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)