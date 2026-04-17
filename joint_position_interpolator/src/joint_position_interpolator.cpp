#include "joint_position_interpolator/joint_position_interpolator.hpp"

namespace joint_controllers
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointPositionInterpolator::JointPositionInterpolator() : ControllerInterface()
  {
  }

  JointPositionInterpolator::~JointPositionInterpolator() = default;

  controller_interface::InterfaceConfiguration JointPositionInterpolator::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    robot_interface_->set_commands_names();

    config.names = robot_interface_->get_commands_names();
    return config;
  }

  controller_interface::InterfaceConfiguration JointPositionInterpolator::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    robot_interface_->set_states_names();

    config.names = robot_interface_->get_states_names();
    return config;
  }

  void JointPositionInterpolator::loadParameters()
  {
    declare_and_get_parameters("max_velocity", max_interpolation_velocity_, 0.01);
    declare_and_get_parameters("joint_names", joint_names_, std::vector<std::string>{});
  }

  void JointPositionInterpolator::setupSubscribers()
  {
    desired_position_sub_ =
        get_node()->create_subscription<extender_msgs::msg::JointPositionCommand>(
            "/joint_position_desired", 10,
            std::bind(&JointPositionInterpolator::desiredPositionCallback, this,
                      std::placeholders::_1));
  }

  void JointPositionInterpolator::setupPublishers()
  {
  }
  bool JointPositionInterpolator::setupRobotInterface()
  {
    auto node = get_node();

    std::string robot_description;
    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to find 'robot_description' parameter.");
      return false;
    }

    robot_interface_ = std::make_unique<robot_interfaces::GenericJointPosition>(joint_names_);
    if (!robot_interface_ || !robot_interface_->initKinematics(
                                 robot_description, node->get_parameter("tool_frame").as_string()))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot interface.");
      return false;
    }

    lowerLimits = robot_interface_->getLowerPositionJointLimits();
    upperLimits = robot_interface_->getUpperPositionJointLimits();
    velLimits = robot_interface_->getVelocityJointLimits();
  }

  void JointPositionInterpolator::activatePublishers()
  {
  }
  void JointPositionInterpolator::deactivatePublishers()
  {
  }

  CallbackReturn JointPositionInterpolator::on_init()
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_configure(const rclcpp_lifecycle::State &)
  {
    // Create robot interface
    if (!setupRobotInterface())
    {
      return CallbackReturn::ERROR;
    }

    loadParameters();
    setupSubscribers();
    setupPublishers();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_interface_->assign_loaned_command(command_interfaces_);
    robot_interface_->assign_loaned_state(state_interfaces_);

    activatePublishers();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_interface_->release_all_interfaces();
    deactivatePublishers();

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointPositionInterpolator::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    robot_interface_->syncState();

    if (!trajectory_running_)
    {
      return controller_interface::return_type::OK;
    }

    elapsed_time_ += period.seconds();
    double t = std::min(elapsed_time_ / trajectory_duration_, 1.0);

    // Linear Interpolation: p = p0 + t * (p1 - p0)
    Eigen::VectorXd commanded_pos = start_positions_ + t * (target_positions_ - start_positions_);

    std::vector<double> cmd_vec(commanded_pos.data(), commanded_pos.data() + commanded_pos.size());

    robot_interfaces::JointCommand jcommand;
    jcommand.command = cmd_vec;
    robot_interface_->setCommand(jcommand);

    if (t >= 1.0)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory complete.");
      trajectory_running_ = false;
    }

    return controller_interface::return_type::OK;
  }

  void JointPositionInterpolator::desiredPositionCallback(
      const extender_msgs::msg::JointPositionCommand::SharedPtr msg)
  {
    if (trajectory_running_)
      return;

    target_positions_map_.clear();
    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      target_positions_map_[msg->joint_names[i]] = msg->desired_position[i];
    }
    computeTrajectory();
  }

  void JointPositionInterpolator::computeTrajectory()
  {
    size_t num_joints = joint_names_.size();
    Eigen::VectorXd current_q = robot_interface_->getJointPositions();

    start_positions_ = current_q;
    target_positions_ = current_q; // Default: stay where we are

    for (size_t i = 0; i < num_joints; ++i)
    {
      auto it = target_positions_map_.find(joint_names_[i]);
      if (it != target_positions_map_.end())
      {
        target_positions_[i] = it->second;
      }
    }

    Eigen::VectorXd diff = target_positions_ - start_positions_;
    Eigen::VectorXd effective_vel_limits = velLimits.cwiseMin(max_interpolation_velocity_);

    // duration = max( |distance| / velocity_limit )
    Eigen::VectorXd times = diff.array().abs() / effective_vel_limits.array();
    trajectory_duration_ = times.maxCoeff();

    if (trajectory_duration_ < 1e-6)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Target already reached.");
      trajectory_running_ = false;
      return;
    }

    elapsed_time_ = 0.0;
    trajectory_running_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "Trajectory started: %f seconds", trajectory_duration_);
  }

} // namespace joint_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(joint_controllers::JointPositionInterpolator,
                       controller_interface::ControllerInterface)
