#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"
#include "joystick_interface/msg/teleop_cmd.hpp"

namespace cartesian_velocity_controller
{
  /// @brief A ROS2 controller that subscribes to Twist commands (from teleop node)
  /// and sends Cartesian velocity commands.
  class CartesianVelocityTeleopController : public controller_interface::ControllerInterface
  {
  public:
    CartesianVelocityTeleopController();
    virtual ~CartesianVelocityTeleopController();

    // Configure command and state interfaces.
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager.
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks.
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

    /// Callback to receive Twist commands from the teleop node.
    void twistCallback(const joystick_interface::msg::TeleopCmd::SharedPtr msg);

    /// Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    /// Stores the latest Twist command received.
    geometry_msgs::msg::Twist latest_twist_;

    /// Stores the latest Twist filtered.
    geometry_msgs::msg::Twist smoothed_twist_;

    /// Subscription for teleop Twist commands.
    rclcpp::Subscription<joystick_interface::msg::TeleopCmd>::SharedPtr twist_sub_;

    /// Overall scaling of the computed velocity command.
    double gain_;

    /// Filter and saturation parameters.
    double alpha_;
    double max_linear_delta_;
    double max_angular_delta_;
  };
} // namespace cartesian_velocity_controller