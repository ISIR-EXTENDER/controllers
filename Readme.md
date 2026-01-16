# Controllers

This repository contains ROS2 controller packages for the extender project, providing various control strategies for robot manipulators including teleoperation, trajectory execution, and shared control paradigms.

## Available Controllers

### Cartesian Velocity Controller
**Package**: `cartesian_velocity`

A ROS2 controller that provides teleoperation control for robot manipulators in Cartesian space. This controller subscribes to teleoperation commands (typically from a joystick interface) and converts them into Cartesian velocity commands for smooth robot control.

**Use Cases**: Direct teleoperation, joystick control, velocity-based manipulation tasks.

---

### Joint Position Interpolator
**Package**: `joint_position_interpolator`

A ROS2 controller that provides smooth joint-space trajectory interpolation for robot manipulators. The controller computes and executes trajectories from the current joint configuration to desired target positions with configurable velocity limits.

**Use Cases**: Point-to-point motion, joint-space trajectory execution, safe joint positioning.

---

### Kinematic Guides Cartesian Velocity Controller
**Package**: `kinematic_guides_cartesian_velocity`

A ROS2 controller that implements shared control teleoperation for robot manipulators with kinematic guidance assistance. This controller blends user input with goal-directed assistance to provide intuitive and efficient teleoperation while maintaining user agency.

**Use Cases**: Assisted teleoperation, goal-directed manipulation, human-robot collaboration tasks.

## Contributing

When adding new controllers to this repository:

1. Follow the established naming and structure conventions
2. Include comprehensive documentation (README, parameter descriptions)
3. Provide launch files for common use cases
4. Add appropriate tests and validation
5. Update this global README with the new controller information
