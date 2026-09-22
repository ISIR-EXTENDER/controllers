#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "extender_msgs/msg/teleop_command.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"
#include "std_msgs/msg/bool.hpp"

namespace cartesian_velocity_controller
{
  /// @brief Snake Demo Controller
  class SnakeDemoTeleopController : public controller_interface::ControllerInterface
  {
  public:
    SnakeDemoTeleopController();
    virtual ~SnakeDemoTeleopController();

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
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      auto node = get_node();
      if (!node->has_parameter(name))
      {
        node->declare_parameter(name, default_value);
      }
      variable = node->get_parameter(name).get_value<T>();
    }

    void loadParameters();

    void declareSubscribers();

    void declarePublishers();

    bool setupRobotInterface();

    /// Callback to receive Twist commands from the teleop node
    void twistCallback(const extender_msgs::msg::TeleopCommand::SharedPtr msg);
    void snakeCallback(const std_msgs::msg::Bool::SharedPtr msg);

    Eigen::Vector3d computeSnakeVelocity(const Eigen::Matrix3d &current_orientation,
                                         const Eigen::Vector3d &linear_velocity,
                                         const Eigen::Vector3d &angular_velocity_) const;
    void applyDeadzone(Eigen::Vector3d &current_command) const;

        // Helper: apply simple per-axis rate limiting (saturation) to linear and angular velocities
        std::pair<Eigen::Vector3d, Eigen::Vector3d> applyVelocitySaturation(
            const Eigen::Vector3d &current_linear, const Eigen::Vector3d &current_angular) const;

    /// Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    /// Stores the latest Twist command received (raw from teleop)
    geometry_msgs::msg::Twist latest_twist_;
    int mode_;
    bool snake_active = false;

    /// Subscription for teleop Twist commands
    rclcpp::Subscription<extender_msgs::msg::TeleopCommand>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr snake_sub_;

    // Robot state
    Eigen::Quaterniond current_orientation_;

    /// Overall scaling of the computed velocity command
    double gain_;

    // Snake scaling of the rotation over the translation
    double snake_gain_;

    /// Cartesian velocity saturation parameters
    double v_max_;
    double omega_max_;
    double deadzone;

    std::vector<std::string> command_names_;
    std::string robot_type_{"franka_velocity"};
  };
} // namespace cartesian_velocity_controller