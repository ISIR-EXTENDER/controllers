#include "cartesian_velocity/snake_demo.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cartesian_velocity_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SnakeDemoTeleopController::SnakeDemoTeleopController()
      : ControllerInterface(), latest_twist_(), current_orientation_(Eigen::Quaterniond::Identity())
  {
  }

  SnakeDemoTeleopController::~SnakeDemoTeleopController() = default;

  controller_interface::InterfaceConfiguration SnakeDemoTeleopController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_vel_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration SnakeDemoTeleopController::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = robot_vel_interface_->get_states_names();
    return config;
  }

  CallbackReturn SnakeDemoTeleopController::on_init()
  {
    declareSubscribers();
    return CallbackReturn::SUCCESS;
  }

  void SnakeDemoTeleopController::loadParameters()
  {
    declare_and_get_parameters("gain", gain_, 1.0);
    declare_and_get_parameters("snake_gain", snake_gain_, 3.0);
    declare_and_get_parameters("max_linear", v_max_, 0.07);
    declare_and_get_parameters("max_angular", omega_max_, 0.014);
    declare_and_get_parameters("deadzone_joy", deadzone, 0.2);
    declare_and_get_parameters("robot_type", robot_type_, std::string("franka_velocity"));
    declare_and_get_parameters("command_names", command_names_, std::vector<std::string>{});
  }

  void SnakeDemoTeleopController::declareSubscribers()
  {
    twist_sub_ = get_node()->create_subscription<extender_msgs::msg::TeleopCommand>(
        "/teleop_cmd", 10,
        std::bind(&SnakeDemoTeleopController::twistCallback, this, std::placeholders::_1));

    snake_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "/activate_snake", 10,
        std::bind(&SnakeDemoTeleopController::snakeCallback, this, std::placeholders::_1));
  }

  void SnakeDemoTeleopController::declarePublishers()
  {
    // No publishers for now
  }

  bool SnakeDemoTeleopController::setupRobotInterface()
  {
    auto node = get_node();
    std::string robot_description;

    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Missing robot_description");
      return false;
    }

    robot_vel_interface_ = robot_interfaces::create_robot_component(robot_type_);
    if (!robot_vel_interface_ ||
        !robot_vel_interface_->initKinematics(robot_description,
                                              node->get_parameter("tool_frame").as_string()))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }
    robot_vel_interface_->set_commands_names(command_names_);

    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  SnakeDemoTeleopController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    loadParameters();
    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SnakeDemoTeleopController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the Franka Cartesian velocity interface
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    robot_vel_interface_->assign_loaned_state(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SnakeDemoTeleopController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type SnakeDemoTeleopController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    robot_vel_interface_->syncState();

    typedef extender_msgs::msg::TeleopCommand Mode;
    // Get current EE pose.
    robot_interfaces::CartesianPosition temp_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();

    current_orientation_ = temp_pose.quaternion;

    Eigen::Vector3d raw_linear;
    Eigen::Vector3d raw_angular;

    switch (mode_)
    {
    case Mode::TRANSLATION_ROTATION:
      raw_linear << latest_twist_.linear.y, latest_twist_.linear.x, 0.;
      raw_angular.setZero();
      break;
    case Mode::BOTH:
      raw_linear << 0., 0., -latest_twist_.linear.y;
      raw_angular << 0.0, 0.0, latest_twist_.linear.x;
      applyDeadzone(raw_linear);
      applyDeadzone(raw_angular);
      break;
    }
    // Apply teleop mode selection on the filtered Cartesian velocities
    Eigen::Vector3d cartesian_linear_velocity = raw_linear;
    Eigen::Vector3d cartesian_angular_velocity = raw_angular;

    const Eigen::Matrix3d R_BE = current_orientation_.toRotationMatrix();
    cartesian_angular_velocity = R_BE * cartesian_angular_velocity;

    // Apply overall gain
    cartesian_linear_velocity *= v_max_;
    cartesian_angular_velocity *= omega_max_;

    if (snake_active)
    {
      cartesian_angular_velocity =
          computeSnakeVelocity(current_orientation_.toRotationMatrix(), cartesian_linear_velocity,
                               cartesian_angular_velocity);
    }

    std::tie(cartesian_linear_velocity, cartesian_angular_velocity) =
        applyVelocitySaturation(cartesian_linear_velocity, cartesian_angular_velocity);
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

  std::pair<Eigen::Vector3d, Eigen::Vector3d> SnakeDemoTeleopController::applyVelocitySaturation(
      const Eigen::Vector3d &current_linear, const Eigen::Vector3d &current_angular) const
  {
    Eigen::Vector3d saturated_linear = current_linear;
    Eigen::Vector3d saturated_angular = current_angular;

    // Saturate linear velocity
    const double linear_norm = saturated_linear.norm();
    if (linear_norm > v_max_)
    {
      saturated_angular *= (v_max_ / linear_norm);
      saturated_linear *= (v_max_ / linear_norm);
    }

    // Saturate angular velocity
    const double angular_norm = saturated_angular.norm();
    if (angular_norm > omega_max_)
    {
      saturated_linear *= (omega_max_ / angular_norm);
      saturated_angular *= (omega_max_ / angular_norm);
    }

    return {saturated_linear, saturated_angular};
  }

  void SnakeDemoTeleopController::applyDeadzone(Eigen::Vector3d &current_command) const
  {
    for (size_t i = 0; i < 3; i++)
    {
      double x = current_command[i];
      if (std::abs(x) < deadzone)
        current_command[i] = 0.0;
      else
        current_command[i] = std::copysign((std::abs(x) - deadzone) / (1.0 - deadzone), x);
    }
  }

  Eigen::Vector3d SnakeDemoTeleopController::computeSnakeVelocity(
      const Eigen::Matrix3d &current_orientation, const Eigen::Vector3d &linear_velocity,
      const Eigen::Vector3d &angular_velocity_) const
  {
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d z7 = current_orientation.col(2);
    angular_velocity = snake_gain_ * z7.cross(linear_velocity) + angular_velocity_;

    return angular_velocity;
  }

  void SnakeDemoTeleopController::twistCallback(
      const extender_msgs::msg::TeleopCommand::SharedPtr msg)
  {
    // Update the stored Twist command with the latest message
    latest_twist_ = msg->twist;
    mode_ = msg->mode;
  }

  void SnakeDemoTeleopController::snakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // Update the stored Twist command with the latest message
    snake_active = msg->data;
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::SnakeDemoTeleopController,
                       controller_interface::ControllerInterface)
