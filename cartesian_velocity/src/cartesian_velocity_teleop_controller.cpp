#include "cartesian_velocity/cartesian_velocity_teleop_controller.hpp"

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
    twist_sub_ = get_node()->create_subscription<joystick_interface::msg::TeleopCmd>(
        "/teleop_cmd", 10,
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
    if (!get_node()->has_parameter("initial_filter_cutoff_frequency"))
    {
      // Same name/semantics as in SharedControlVelocityController
      get_node()->declare_parameter("initial_filter_cutoff_frequency", 0.8);
    }
    if (!get_node()->has_parameter("max_linear_delta"))
    {
      get_node()->declare_parameter("max_linear_delta", 0.007);
    }
    if (!get_node()->has_parameter("max_angular_delta"))
    {
      get_node()->declare_parameter("max_angular_delta", 0.014);
    }

    // Read controller and filter parameters from server.
    gain_ = get_node()->get_parameter("gain").as_double();
    initial_filter_cutoff_frequency_ =
        get_node()->get_parameter("initial_filter_cutoff_frequency").as_double();
    max_linear_delta_ = get_node()->get_parameter("max_linear_delta").as_double();
    max_angular_delta_ = get_node()->get_parameter("max_angular_delta").as_double();

    // Print the parameters on console.
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Velocity Controller - gain: %.4f", gain_);
    RCLCPP_INFO(get_node()->get_logger(),
                "Cartesian Velocity Controller - initial_filter_cutoff_frequency: %.4f",
                initial_filter_cutoff_frequency_);
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

    // Reset LPF state
    smoothed_twist_ = geometry_msgs::msg::Twist{};
    lpf_initialized_ = false;

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
      const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    // Start from the latest commanded twist and apply teleop mode selection.
    geometry_msgs::msg::Twist mode_filtered_twist = latest_twist_;

    switch (mode_)
    {
    case TeleopMode::Translation:
      // Only translation: ignore angular component.
      mode_filtered_twist.angular.x = 0.0;
      mode_filtered_twist.angular.y = 0.0;
      mode_filtered_twist.angular.z = 0.0;
      break;
    case TeleopMode::Rotation:
      // Only rotation: ignore linear component.
      mode_filtered_twist.linear.x = 0.0;
      mode_filtered_twist.linear.y = 0.0;
      mode_filtered_twist.linear.z = 0.0;
      break;
    case TeleopMode::Both:
    case TeleopMode::Translation_Rotation:
    default:
      // Use both linear and angular components.
      break;
    }

    // Compute alpha from desired cutoff frequency: alpha = T / (T + tau)
    const double dt_sec = period.seconds();
    double alpha = 1.0; // default: passthrough if fc <= 0 or dt <= 0
    if (initial_filter_cutoff_frequency_ > 0.0 && dt_sec > 0.0)
    {
      const double tau = 1.0 / (2.0 * M_PI * initial_filter_cutoff_frequency_);
      alpha = std::clamp(dt_sec / (dt_sec + tau), 1e-9, 1.0 - 1e-9);
    }

    // On first run, initialise smoothed_twist_ to avoid startup transients.
    if (!lpf_initialized_)
    {
      smoothed_twist_ = mode_filtered_twist;
      lpf_initialized_ = true;
    }

    // Save the previous smoothed twist values to check the rate of change.
    geometry_msgs::msg::Twist previous_smoothed = smoothed_twist_;

    // Apply a low-pass filter to the latest twist command.
    // y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    smoothed_twist_.linear.x = alpha * mode_filtered_twist.linear.x +
                               (1.0 - alpha) * smoothed_twist_.linear.x;
    smoothed_twist_.linear.y = alpha * mode_filtered_twist.linear.y +
                               (1.0 - alpha) * smoothed_twist_.linear.y;
    smoothed_twist_.linear.z = alpha * mode_filtered_twist.linear.z +
                               (1.0 - alpha) * smoothed_twist_.linear.z;

    smoothed_twist_.angular.x = alpha * mode_filtered_twist.angular.x +
                                (1.0 - alpha) * smoothed_twist_.angular.x;
    smoothed_twist_.angular.y = alpha * mode_filtered_twist.angular.y +
                                (1.0 - alpha) * smoothed_twist_.angular.y;
    smoothed_twist_.angular.z = alpha * mode_filtered_twist.angular.z +
                                (1.0 - alpha) * smoothed_twist_.angular.z;

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

    robot_interfaces::CartesianVelocity vel_cmd;
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
      const joystick_interface::msg::TeleopCmd::SharedPtr msg)
  {
    // Update the stored Twist command with the latest message.
    latest_twist_ = msg->twist;

    // Store the teleoperation mode for use in update().
    switch (msg->mode)
    {
    case joystick_interface::msg::TeleopCmd::TRANSLATION_ROTATION:
      mode_ = TeleopMode::Translation_Rotation;
      break;
    case joystick_interface::msg::TeleopCmd::ROTATION:
      mode_ = TeleopMode::Rotation;
      break;
    case joystick_interface::msg::TeleopCmd::TRANSLATION:
      mode_ = TeleopMode::Translation;
      break;
    case joystick_interface::msg::TeleopCmd::BOTH:
      mode_ = TeleopMode::Both;
      break;
    default:
      mode_ = TeleopMode::Translation_Rotation;
      break;
    }
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::CartesianVelocityTeleopController,
                       controller_interface::ControllerInterface)
