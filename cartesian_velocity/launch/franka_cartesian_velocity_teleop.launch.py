import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_robot_description(context, robot_ip, arm_id, use_fake_hardware, fake_sensor_commands, load_gripper):
    arm_id_str = context.perform_substitution(arm_id)
    robot_ip_str = context.perform_substitution(robot_ip)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'ros2_control': 'true',
            'arm_id': arm_id_str,
            'robot_ip': robot_ip_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,  # Should be "false" for real robot
            'fake_sensor_commands': fake_sensor_commands_str,
        }
    )
    robot_description = robot_description_config.toprettyxml(indent='  ')
    return robot_description


def launch_setup(context, *args, **kwargs):
    # Retrieve launch configurations.
    robot_ip = LaunchConfiguration('robot_ip')
    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')  # Set to false for real robot
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    # Generate the robot description from the xacro.
    robot_description = generate_robot_description(
        context, robot_ip, arm_id, use_fake_hardware, fake_sensor_commands, load_gripper
    )



    nodes = []

    # --- Robot Bringup Nodes ---
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    ))

    # Standard Franka controller configuration.
    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
    )
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            franka_controllers,
            {'robot_description': robot_description},
            {'arm_id': arm_id},
            {'load_gripper': load_gripper},
        ],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    ))

    nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'], 'rate': 30}],
    ))

    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    ))

    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        parameters=[{'arm_id': arm_id}],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    ))

    # --- Teleoperation Node ---
    teleop_config_file = os.path.join(
        get_package_share_directory('joystick_interface'),
        'config',
        'franka_joystick_parameters.yaml'
    )
    nodes.append(Node(
        package='joystick_interface',
        executable='joystick_input_node',
        name='joystick_input_node',
        output='screen',
        parameters=[teleop_config_file],
    ))

    # --- Gripper Node ---
    gripper_config_file = os.path.join(
        get_package_share_directory('joystick_interface'),
        'config',
        'franka_gripper_parameters.yaml'
    )
    nodes.append(Node(
        package='joystick_interface',
        executable='franka_gripper_node',
        name='franka_gripper_node',
        parameters=[gripper_config_file],
    ))


    # --- Cartesian Velocity Controller ---
    custom_controller_config = PathJoinSubstitution([
        FindPackageShare('cartesian_velocity'),
        'config',
        'franka_cartesian_teleop.yaml'
    ])

    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "cartesian_velocity_teleop_controller",
            "-c", "controller_manager",
            "-t", "cartesian_velocity/CartesianVelocityTeleopController",
            "--param-file", custom_controller_config
        ],
        output='screen'
    ))

    return nodes

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            'arm_id',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware (set to false for real robot)'
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Fake sensor commands. Only valid when use_fake_hardware is true'
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.'
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
