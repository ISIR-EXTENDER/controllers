# Cartesian Velocity Controller

A ROS2 controller package that provides teleoperation control for robot manipulators in Cartesian space. This controller subscribes to teleoperation commands (typically from a joystick interface) and converts them into Cartesian velocity commands for smooth robot control.

## Overview

The Cartesian Velocity Controller enables intuitive teleoperation of robot arms by translating user input (such as joystick commands) into Cartesian space velocities. The controller features:

- **Multi-robot support**: Compatible with Franka Emika Panda, Kinova Gen3, and custom robot interfaces
- **Filtering and smoothing**: Low-pass filtering and rate limiting for stable, smooth motion
- **Flexible teleoperation modes**: Support for translation-only, rotation-only, or combined motion
- **Configurable parameters**: Adjustable gains, filter frequencies, and velocity limits
- **Frame support**: Commands can be interpreted in base frame or end-effector frame

## Dependencies

- ROS2 Humble or later
- `controller_interface`
- `pluginlib`
- `geometry_msgs`
- `robot_interfaces` (custom package)
- `joystick_interface` (custom package)
- Eigen3

## Configuration

The controller is configured through ROS2 parameters. Example configuration files are provided in the `config/` directory for different robot types:

- `franka_params.yaml` - Configuration for Franka Emika Panda robots
- `kinova_params.yaml` - Configuration for Kinova Gen3 robots
- `explorer_params.yaml` - Configuration for custom Explorer robots

### Key Parameters

- `gain`: Overall scaling factor for velocity commands (default: 1.0)
- `initial_filter_cutoff_frequency`: Cutoff frequency for low-pass filter in Hz (default: 0.8)
- `max_linear_delta`: Maximum change in linear velocity per control cycle (default: 0.007)
- `max_angular_delta`: Maximum change in angular velocity per control cycle (default: 0.014)
- `robot_type`: Type of robot interface ("franka_velocity", "kinova_velocity", "explorer_velocity")
- `input_twist_frame`: Reference frame for input commands ("base" or "ee")
- `base_frame`: Name of the robot's base frame
- `tool_frame`: Name of the robot's tool/end-effector frame

## Usage

### Launch Files

The package includes launch files for different robot configurations:

- `franka_cartesian_velocity_teleop.launch.py` - Launch with Franka robot
- `kinova_gen3_teleop_full_bringup.launch.py` - Full bringup with Kinova Gen3
- `full_gen3_teleop_bringup.launch.py` - Alternate full bringup for Kinova Gen3
- `explorer_cartesian_velocity_teleop.launch.py` - Launch with Explorer robot

### Example Launch

For Explorer robots:
```bash
ros2 launch cartesian_velocity explorer_cartesian_velocity_teleop.launch.py \
  can_port:='can0'
```

### Input Interface Selection

Teleop launch files expose:

- `use_joystick_interface:=true|false` (default: `false`)

When `true`, launch starts joystick input nodes (`joystick_input_node` and `joy_node`).
When `false`, joystick nodes are not started, which is useful when commands come from another UI/source.

## Teleoperation

### Interface

The controller subscribes to `/teleop_cmd` topic of type `joystick_interface/msg/TeleopCmd`, which contains:

- `twist`: The desired Cartesian velocity (linear and angular components)
- `mode`: Teleoperation mode (TRANSLATION, ROTATION, BOTH, or TRANSLATION_ROTATION)

### Modes

- **TRANSLATION**: Only linear velocity commands are applied
- **ROTATION**: Only angular velocity commands are applied
- **BOTH/TRANSLATION_ROTATION**: Both linear and angular velocities are applied

### TODO 

- Add jaco mode 
