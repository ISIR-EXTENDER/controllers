#include "shared_control_icra_2025/cartesian_velocity_teleop_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cartesian_velocity_controller
{
  // Helper function to clamp a value within a range.
  inline double clamp_value(double value, double min_val, double max_val)
  {
    return std::clamp(value, min_val, max_val);
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CartesianVelocityTeleopController::CartesianVelocityTeleopController()
      : ControllerInterface(), latest_twist_()
  {
    // Initialize twist command to zero.
    latest_twist_.linear.x = 0.0;
    latest_twist_.linear.y = 0.0;
    latest_twist_.linear.z = 0.0;
    latest_twist_.angular.x = 0.0;
    latest_twist_.angular.y = 0.0;
    latest_twist_.angular.z = 0.0;

    smoothed_twist_ = latest_twist_;
  }

  CartesianVelocityTeleopController::~CartesianVelocityTeleopController()
  {
  }

  controller_interface::InterfaceConfiguration CartesianVelocityTeleopController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    robot_vel_interface_->set_commands_names();

    config.names = robot_vel_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration CartesianVelocityTeleopController::
      state_interface_configuration() const
  {
    // This controller does not use state interfaces.
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
  }

  CallbackReturn CartesianVelocityTeleopController::on_init()
  {
    // Create a subscription to the "cmd_vel" topic to receive teleoperation commands.
    twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CartesianVelocityTeleopController::twistCallback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CartesianVelocityTeleopController::on_configure(const rclcpp_lifecycle::State &)
  {
    // Check and declare parameters only if not already declared.
    if (!get_node()->has_parameter("gain"))
    {
      get_node()->declare_parameter("gain", 1.0);
    }
    if (!get_node()->has_parameter("alpha"))
    {
      get_node()->declare_parameter("alpha", 0.005);
    }
    if (!get_node()->has_parameter("max_linear_delta"))
    {
      get_node()->declare_parameter("max_linear_delta", 0.007);
    }
    if (!get_node()->has_parameter("max_angular_delta"))
    {
      get_node()->declare_parameter("max_angular_delta", 0.014);
    }

    // Read controller and filter parameter from server.
    gain_ = get_node()->get_parameter("gain").as_double();
    alpha_ = get_node()->get_parameter("alpha").as_double();
    max_linear_delta_ = get_node()->get_parameter("max_linear_delta").as_double();
    max_angular_delta_ = get_node()->get_parameter("max_angular_delta").as_double();

    // Print the parameters on console.
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Velocity Controller - gain: %.4f", gain_);
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Velocity Controller - alpha: %.4f", alpha_);
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Velocity Controller - max_linear_delta: %.4f",
                max_linear_delta_);
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Velocity Controller - max_angular_delta: %.4f",
                max_angular_delta_);

    std::string robot_type = get_node()->get_parameter("robot_type").as_string();
    robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type);
    if (!robot_vel_interface_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create robot interface for type '%s'",
                   robot_type.c_str());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CartesianVelocityTeleopController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the Franka Cartesian velocity interface.
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CartesianVelocityTeleopController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type CartesianVelocityTeleopController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    // Save the previous smoothed twist values to check the rate of change.
    geometry_msgs::msg::Twist previous_smoothed = smoothed_twist_;

    // Apply a low-pass filter to the latest twist command.
    // y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    const double alpha = 0.005; // Smoothing factor

    smoothed_twist_.linear.x =
        alpha_ * latest_twist_.linear.x + (1.0 - alpha_) * smoothed_twist_.linear.x;
    smoothed_twist_.linear.y =
        alpha_ * latest_twist_.linear.y + (1.0 - alpha_) * smoothed_twist_.linear.y;
    smoothed_twist_.linear.z =
        alpha_ * latest_twist_.linear.z + (1.0 - alpha_) * smoothed_twist_.linear.z;

    smoothed_twist_.angular.x =
        alpha_ * latest_twist_.angular.x + (1.0 - alpha_) * smoothed_twist_.angular.x;
    smoothed_twist_.angular.y =
        alpha_ * latest_twist_.angular.y + (1.0 - alpha_) * smoothed_twist_.angular.y;
    smoothed_twist_.angular.z =
        alpha_ * latest_twist_.angular.z + (1.0 - alpha_) * smoothed_twist_.angular.z;

    // Rate limiting: limit the velocity change between cycles to avoid discontinuties.
    // For each component, compute the difference and clamp if necessary.
    smoothed_twist_.linear.x = previous_smoothed.linear.x +
                               clamp_value(smoothed_twist_.linear.x - previous_smoothed.linear.x,
                                           -max_linear_delta_, max_linear_delta_);
    smoothed_twist_.linear.y = previous_smoothed.linear.y +
                               clamp_value(smoothed_twist_.linear.y - previous_smoothed.linear.y,
                                           -max_linear_delta_, max_linear_delta_);
    smoothed_twist_.linear.z = previous_smoothed.linear.z +
                               clamp_value(smoothed_twist_.linear.z - previous_smoothed.linear.z,
                                           -max_linear_delta_, max_linear_delta_);

    smoothed_twist_.angular.x = previous_smoothed.angular.x +
                                clamp_value(smoothed_twist_.angular.x - previous_smoothed.angular.x,
                                            -max_angular_delta_, max_angular_delta_);
    smoothed_twist_.angular.y = previous_smoothed.angular.y +
                                clamp_value(smoothed_twist_.angular.y - previous_smoothed.angular.y,
                                            -max_angular_delta_, max_angular_delta_);
    smoothed_twist_.angular.z = previous_smoothed.angular.z +
                                clamp_value(smoothed_twist_.angular.z - previous_smoothed.angular.z,
                                            -max_angular_delta_, max_angular_delta_);

    // Use the smoothed twist for computing Cartesian velocities.
    Eigen::Vector3d cartesian_linear_velocity(gain_ * smoothed_twist_.linear.x,
                                              gain_ * smoothed_twist_.linear.y,
                                              gain_ * smoothed_twist_.linear.z);
    Eigen::Vector3d cartesian_angular_velocity(gain_ * smoothed_twist_.angular.x,
                                               gain_ * smoothed_twist_.angular.y,
                                               gain_ * smoothed_twist_.angular.z);

    robot_interfaces::CartesianVelocityCommand vel_cmd;
    vel_cmd.linear = cartesian_linear_velocity;
    vel_cmd.angular = cartesian_angular_velocity;
    if (robot_vel_interface_->setCommand(vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  void CartesianVelocityTeleopController::twistCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Update the stored Twist command with the latest message.
    latest_twist_ = *msg;
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::CartesianVelocityTeleopController,
                       controller_interface::ControllerInterface)
