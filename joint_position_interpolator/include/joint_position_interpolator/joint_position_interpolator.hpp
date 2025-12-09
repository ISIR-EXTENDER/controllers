#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

#include "joint_position_interpolator/msg/joint_position_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cmath>

#include <urdf/model.h>

namespace joint_controllers
{
  /// @brief struct to hold useful informations to interpolate
  struct InterpolateData
  {
    /// @brief start position of the joint
    double start_pos;
    /// @brief adjusted end position (unwrapped if joint is continuous)
    double end_pos;
    /// @brief velocity for the interpolation
    double velocity;
  };

  /// @brief Enumeration of joint types that are available in urdf format
  enum class JointType
  {
    UNKNOWN,
    REVOLUTE,   // Rotates with limits (hinge)
    CONTINUOUS, // Rotates without limits (wheel)
    PRISMATIC,  // Slides (linear)
    FIXED,      // No movement
    FLOATING,   // 6DOF
    PLANAR      // Moves in a 2D plane
  };

  /// @brief Struct to hold joint limits of the robot model
  struct JointLimits
  {
    double min_position = -1e9;
    double max_position = 1e9;
    double max_velocity = 0.0;
    bool has_position_limits = false;
    bool has_velocity_limits = false;

    JointType jtype = JointType::UNKNOWN;
  };

  /// @brief Class that implement a joint position interpolator. It allows to compute trajectory
  /// from the current position, to a desired position, with a constant velocity.
  class JointPositionInterpolator : public controller_interface::ControllerInterface
  {
  public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    typedef robot_interfaces::JointCommand JointCommand;
    typedef joint_position_interpolator::msg::JointPositionCommand JointPositionMessage;

    JointPositionInterpolator();
    virtual ~JointPositionInterpolator();

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  private:
    /// @brief Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_interface_;

    /// @brief Subscriber to get the desired position
    rclcpp::Subscription<JointPositionMessage>::SharedPtr
        desired_position_sub_;
    /// @brief Callback of the subscriber. Will store the desired position in the unordered_map target_positions_
    /// @param msg message send on the topic /desired_joint_positions
    void desiredPositionCallback(
        const JointPositionMessage::SharedPtr msg);

    /// @brief Compute all joint trajectories based on current state and desired position
    void computeTrajectory();

    /// @brief Read the topic /robot_description and parse the joint limits
    /// @return false if the topic cannot be subsribed to, true otherwise/
    bool parseLimits();

    /// @brief Interpolation velocity
    double max_interpolation_velocity_ = 0.01; // rad/s, default value

    /// @brief target positions matched with the joint names
    std::unordered_map<std::string, double> target_positions_;
    /// @brief joint names of the robot 
    std::vector<std::string> joint_names_;
    /// @brief Limits of each joints, ordered the same as joint_names
    std::vector<JointLimits> joint_limits_;
    /// @brief Trajectory from start position to desired position
    std::vector<JointCommand> joint_trajectory;
    /// @brief check if trajectory is done or not. For now, as long as a goal is not reached no new goal can be sent
    bool done = true;
    /// @brief current index in the trajectory
    size_t current_index = 0;
  };
} // namespace joint_controllers