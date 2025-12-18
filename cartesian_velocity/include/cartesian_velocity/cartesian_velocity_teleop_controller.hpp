#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "joystick_interface/msg/teleop_cmd.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

namespace cartesian_velocity_controller
{
  /// @brief A ROS2 controller that subscribes to Twist commands (from teleop node)
  /// and sends Cartesian velocity commands
  class CartesianVelocityTeleopController : public controller_interface::ControllerInterface
  {
  public:
    CartesianVelocityTeleopController();
    virtual ~CartesianVelocityTeleopController();

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

  private:
    // Teleoperation Mode (aligned with joystick_interface/TeleopCmd constants)
    enum class TeleopMode
    {
      Translation_Rotation,
      Rotation,
      Translation,
      Both
    };

    TeleopMode mode_{TeleopMode::Translation_Rotation};

    /// Map TeleopCmd.mode (uint8) to internal TeleopMode enum
    TeleopMode fromMsgMode(uint8_t mode) const;

    /// Callback to receive Twist commands from the teleop node
    void twistCallback(const joystick_interface::msg::TeleopCmd::SharedPtr msg);

    /// Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    /// Stores the latest Twist command received (raw from teleop)
    geometry_msgs::msg::Twist latest_twist_;

    /// Internal filtered linear and angular velocities (base frame)
    Eigen::Vector3d filtered_linear_;
    Eigen::Vector3d filtered_angular_;

    /// Subscription for teleop Twist commands
    rclcpp::Subscription<joystick_interface::msg::TeleopCmd>::SharedPtr twist_sub_;

    // Robot state
    Eigen::Quaterniond current_orientation_;

    /// Overall scaling of the computed velocity command
    double gain_;

    // Low-pass filter configuration
    // Cutoff frequency [Hz] for the first-order LPF applied to joystick twist
    double initial_filter_cutoff_frequency_;
    // Flag to initialise filter memory on first update
    bool lpf_initialized_{false};

    /// Cartesian velocity saturation parameters
    double max_linear_delta_;
    double max_angular_delta_;

    // Helper: apply first-order low-pass filter to a 3D vector
    Eigen::Vector3d applyLowPassFilterVector(const Eigen::Vector3d &input,
                                             const Eigen::Vector3d &previous, double alpha) const;

    // Helper: apply simple per-axis rate limiting (saturation) to linear and angular velocities
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applyVelocitySaturation(
        const Eigen::Vector3d &current_linear, const Eigen::Vector3d &previous_linear,
        double max_linear_delta, const Eigen::Vector3d &current_angular,
        const Eigen::Vector3d &previous_angular, double max_angular_delta) const;

    // Frame for interpreting incoming twist commands: "base" or "ee"
    std::string input_twist_frame_{"base"};
  };
} // namespace cartesian_velocity_controller