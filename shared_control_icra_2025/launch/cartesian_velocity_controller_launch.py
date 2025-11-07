from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for the bringup
    robot_ip_parameter_name = 'robot_ip'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # Absolute path to the configuration file for the custom controller
    config_file = PathJoinSubstitution([
        FindPackageShare('shared_control_icra_2025'),
        'config',
        'controllers.yaml'
    ])

    return LaunchDescription([
        # Declare bringup arguments
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in RViz'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(use_fake_hardware_parameter_name)
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Load the gripper as an end-effector, otherwise load without an end-effector.'
        ),

        # Include the main bringup launch file (from franka_bringup)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'),
                    'launch',
                    'franka.launch.py'
                ])
            ]),
            launch_arguments={
                robot_ip_parameter_name: robot_ip,
                arm_id_parameter_name: arm_id,
                load_gripper_parameter_name: load_gripper,
                use_fake_hardware_parameter_name: use_fake_hardware,
                fake_sensor_commands_parameter_name: fake_sensor_commands,
                use_rviz_parameter_name: use_rviz
            }.items(),
        ),

        # Spawn the custom controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["cartesian_velocity_teleop_controller",
                   "-c", "controller_manager",
                   "-t", "shared_control_icra_2025/CartesianVelocityTeleopController",
                   "--param-file", config_file
                  ],
            output='screen',
        ),
    ])
