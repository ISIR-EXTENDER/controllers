# Kinematic Guides Cartesian Velocity Controller

A ROS2 controller package that implements shared control teleoperation for robot manipulators with kinematic guidance assistance. This controller blends user input with goal-directed assistance to provide intuitive and efficient teleoperation while maintaining user agency.

## Overview

The Kinematic Guides Cartesian Velocity Controller enables advanced teleoperation of robot arms by implementing a shared control paradigm. The system provides:

- **Shared Control Assistance**: Blends user commands with autonomous goal-directed behavior
- **Multi-Goal Tracking**: Dynamically selects and tracks multiple predefined goals based on confidence measures
- **Kinematic Guidance**: Provides haptic-like assistance through velocity shaping
- **Visual Feedback**: RViz visualization of goals, confidence levels, and control assistance
- **Multi-Robot Support**: Compatible with Franka Emika Panda, Kinova Gen3, and Explorer robots
- **Flexible Teleoperation Modes**: Support for translation-only, rotation-only, or combined motion

### Goal Representation

Goals are represented as 6D poses (position + orientation) with associated confidence values. The system supports:

- Multiple simultaneous goals
- Dynamic confidence updates based on user behavior
- Soft goal selection through confidence-weighted blending
- Known goals with labels for feedback

## Dependencies

- ROS2 Humble or later
- `controller_interface`
- `pluginlib`
- `rclcpp`, `rclcpp_components`
- `geometry_msgs`, `std_msgs`, `visualization_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `robot_interfaces` (custom package)
- `joystick_interface` (custom package)
- Eigen3

## Configuration

The controller is highly configurable through ROS2 parameters. Configuration files are provided in the `config/` directory for different robot types.

### Core Parameters

#### Control Functions
- `enable_shared_control_assistance`: Enable/disable shared control assistance (default: true)
- `enable_velocity_saturation`: Enable velocity rate limiting (default: true)
- `enable_debug_publish`: Enable debug topic publishing (default: false)

#### Filtering
- `initial_filter_cutoff_frequency`: Low-pass filter cutoff for user input (Hz, default: 2.0)
- `final_filter_cutoff_frequency`: Low-pass filter cutoff for final commands (Hz, default: 5.0)

#### Shared Control Parameters
- `gamma`: Assistance strength parameter (dimensionless, default: 2.0)
- `r1`, `r2`: Distance thresholds for assistance blending (m, default: 0.08, 0.04)
- `theta1_deg`, `theta2_deg`: Angular thresholds for rotational assistance (deg, default: 15.0, 5.0)
- `alpha_conf`: Confidence update rate (1/s, default: 1.5)
- `theta_l_deg`: Angular threshold for confidence updates (deg, default: 15.0)
- `v_j_max`: Maximum joystick velocity for confidence updates (m/s, default: 0.055)

#### Goals Configuration
- `goals_poses`: Array of goal poses as [x,y,z,qx,qy,qz,qw] for each goal
- `known_goals_poses`: Known goals with labels for feedback
- `known_goals_labels`: String labels for known goals

#### Velocity Saturation
- `max_linear_delta`: Maximum linear velocity change per cycle (default: 0.0005)
- `max_angular_delta`: Maximum angular velocity change per cycle (default: 0.0005)

#### Reach Detection
- `eps_t_reach_m`: Translational reach threshold (m, default: 0.01)
- `eps_r_reach_deg`: Rotational reach threshold (deg, default: 10.0)
- `reach_dwell_ms`: Dwell time for reach confirmation (ms, default: 300)

### Robot Configuration
- `robot_type`: Robot interface type ("franka_velocity", "kinova_velocity", "explorer_velocity")
- `base_frame`: Robot base frame name
- `tool_frame`: End-effector frame name
- `input_twist_frame`: Reference frame for input commands ("base" or "ee")

## Usage

### Launch Files

The package includes launch files for different robot configurations:

- `kinematic_guide_franka.launch.py` - Full setup with Franka robot and visualization
- `shared_control_kinova.launch.py` - Setup with Kinova Gen3 robot
- `shared_control_explorer.launch.py` - Setup with custom Explorer robot

### Example Launch

For Franka robots:
```bash
ros2 launch kinematic_guides_cartesian_velocity shared_control_explorer.launch.py \
  can_port:='can0'
```


## Teleoperation 

The controller subscribes to `/teleop_cmd` topic of type `joystick_interface/msg/TeleopCmd`, which contains:

- `twist`: Desired Cartesian velocity (linear and angular components)
- `mode`: Teleoperation mode (TRANSLATION_ROTATION, ROTATION, TRANSLATION, BOTH)

## Visualization

### RViz Integration

The package includes RViz configuration files (`rviz/goal_markers.rviz`) for visualizing:

- **Goal Markers**: Spherical markers representing target goals
- **Confidence Visualization**: Color-coded goal markers based on confidence levels
- **Assistance Cones**: Visual representation of assistance regions
- **Velocity Vectors**: User input and assisted velocity vectors
- **Soft Goal**: Current blended goal position and orientation

### Goal Marker Publisher

A standalone node (`goal_marker_publisher`) provides real-time visualization:

```bash
ros2 run kinematic_guides_cartesian_velocity goal_marker_publisher
```

## Debug Topics

When `enable_debug_publish` is true, the controller publishes debug information:

- `/debug/goal_confidences`: Confidence values for each goal
- `/debug/soft_goal`: Current soft goal pose
- `/debug/agnostic_goal`: Goal without assistance
- `/debug/raw_velocity`: Raw user input velocity
- `/debug/assisted_angular_velocity`: Angular velocity after assistance
- `/debug/final_filtered_linear_velocity`: Final filtered linear velocity
- `/debug/final_filtered_angular_velocity`: Final filtered angular velocity
- `/debug/goal_reached`: Index of reached goal
- `/debug/mode`: Current teleoperation mode
