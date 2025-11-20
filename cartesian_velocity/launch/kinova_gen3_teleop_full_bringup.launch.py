from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # description_package and description_file kept flexible but default to kortex/gen3
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # Robot_description from kortex_description/robots/*.xacro ---
    urdf_cmd = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(description_package),
            "robots",
            description_file
        ]),
        " ",
        "robot_ip:=", robot_ip,
        " ",
        "name:=", "arm",
        " ",
        "arm:=", "gen3",
        " ",
        "dof:=", "7",
        " ",
        "prefix:=", "",
        " ",
        "use_fake_hardware:=", use_fake_hardware,
        " ",
        "fake_sensor_commands:=", fake_sensor_commands,
        " ",
        # we skip gripper-specific xacro args here; they’ll be ignored if unused
    ])
    robot_description = {"robot_description": urdf_cmd}

    # Controllers YAML  
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("cartesian_velocity"),
        "config",
        "kinova_cartesian_teleop_controllers.yaml",
    ])

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
            controllers_yaml,
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

    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
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
            controllers_yaml,
        ],
        output="screen",
    )

    # Joystick config YAML
    joystick_params_yaml = PathJoinSubstitution([
        FindPackageShare("joystick_interface"),
        "config",
        "kinova_joystick_parameters.yaml",
    ])

    # Joystick Interface Node
    joystick_node = Node(
        package="joystick_interface",
        executable="joystick_input_node",
        name="joystick_input",
        output="screen",
        parameters=[joystick_params_yaml],
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "rviz",
        "view_robot.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="log",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
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
            "launch_rviz",
            default_value="false",
            description="Launch RViz?",
        ),

        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        fault_controller_spawner,
        cartesian_teleop_spawner,
        joystick_node,
        rviz_node,
    ])
