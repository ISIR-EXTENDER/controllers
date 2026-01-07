# Joint Position Interpolator

A ROS2 controller package that provides smooth joint-space trajectory interpolation for robot manipulators. The controller computes and executes trajectories from the current joint configuration to desired target positions with configurable velocity limits.

## Overview

The Joint Position Interpolator is a ROS2 controller that enables precise joint-space motion control by:

- **Trajectory Generation**: Computes smooth trajectories from current to desired joint positions
- **Velocity Control**: Respects configurable maximum joint velocities for safe operation
- **Joint Limits Awareness**: Handles different joint types (revolute, continuous, prismatic)
- **Real-time Execution**: Executes pre-computed trajectories at control loop frequency
- **Multi-Robot Support**: Compatible with various robot interfaces through generic abstraction

## Dependencies

- ROS2 Humble or later
- `controller_interface`
- `rclcpp`, `rclcpp_lifecycle`
- `realtime_tools`
- `std_msgs`, `sensor_msgs`
- `robot_interfaces` (custom package)
- Eigen3
- URDF

## Configuration

### Parameters

The controller is configured through ROS2 parameters:

- `joint_names`: Array of joint names to control (required)
- `max_velocity`: Maximum interpolation velocity in rad/s (default: 0.01)
- `robot_description`: URDF robot description string (required)
- `base_frame`: Robot base frame name (required for kinematics)
- `tool_frame`: End-effector frame name (required for kinematics)

### Example Configuration

```yaml
joint_position_interpolator:
  ros__parameters:
    joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    max_velocity: 0.5  # rad/s
    base_frame: "base_link"
    tool_frame: "tool_frame"
```

## Usage

### Launch Files

The package includes launch files for different configurations:

- `explorer_real.launch.py` - Launch with real Explorer robot hardware
- `explorer_sim.launch.py` - Launch with simulated Explorer robot

### Example Launch

```bash
ros2 launch joint_position_interpolator explorer_real.launch.py \
  can_port:='can0'
```
## Interface

### Input Topic

The controller subscribes to `/joint_position_desired` topic with message type `joint_position_interpolator/JointPositionCommand`:

```msg
string[] joint_names      # Names of joints to move
float64[] desired_position # Target positions in radians (or meters for prismatic joints)
```

### Command Example

Send a joint position command using ROS2 CLI:

```bash
ros2 topic pub /joint_position_desired joint_position_interpolator/msg/JointPositionCommand "
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
desired_position: [0.0, 0.5, 0.0, -0.5, 0.0, 0.0]"
```

Or using Python:

```python
import rclcpp
from joint_position_interpolator.msg import JointPositionCommand

# Create message
msg = JointPositionCommand()
msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
msg.desired_position = [0.0, 0.5, 0.0, -0.5, 0.0, 0.0]

# Publish
publisher.publish(msg)
```

## Trajectory Computation

### Joint Type Handling

- **Revolute Joints**: Standard position interpolation
- **Continuous Joints**: Automatic angle unwrapping to prevent multi-turn issues
- **Prismatic Joints**: Linear position interpolation

### Velocity Profile

The controller uses a **constant velocity profile** where:

- All joints move at their individual maximum velocities
- Total trajectory time is determined by the joint requiring the most time
- Slower joints wait at their target positions while faster joints complete their motion

## Limitations

- **Single Trajectory**: Only one trajectory can be active at a time
- **Linear Interpolation**: Uses simple linear interpolation (no acceleration profiles)
- **Memory Usage**: Stores entire trajectory in memory
- **No Online Modification**: Cannot modify trajectory once started

## Future Enhancements

Potential improvements could include:

- Cubic spline interpolation for smoother motion
- Online trajectory modification
- Acceleration/deceleration profiles
- Multi-trajectory queuing
- Trajectory blending for continuous motion