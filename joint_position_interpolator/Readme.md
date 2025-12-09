# Joint Position Interpolator

This ros2 package implement a joint position interpolator. Based on the current state when the desired position is received, it will interpolate a trajectory for each joint in the robot interface based on the max_velocity defined as parameter.

## Component
- JointPositionInterpolator : A ros2 controller 
    - Input : topic /joint_position_desired (joint_position_interpolator/msg/JointPositionCommand)
    
- JointPositionCommand : a ros2 custom message
    - joint_names : string[]
    - desired_position : float64[]