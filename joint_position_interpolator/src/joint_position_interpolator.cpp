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

  CallbackReturn JointPositionInterpolator::on_init()
  {
    desired_position_sub_ =
        get_node()->create_subscription<extender_msgs::msg::JointPositionCommand>(
            "/joint_position_desired", 10,
            std::bind(&JointPositionInterpolator::desiredPositionCallback, this,
                      std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node = get_node();

    std::vector<std::string> joint_names;
    if (!node->get_parameter("joint_names", joint_names))
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter 'joint_names' not set!");
      return CallbackReturn::ERROR;
    }

    // 2. Instantiate directly using the new constructor
    robot_interface_ = std::make_unique<robot_interfaces::GenericJointPosition>(joint_names);

    // init kinematics of the robot interface
    std::string robot_description;
    if (!node->get_parameter("robot_description", robot_description))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to find 'robot_description' parameter.");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!robot_interface_->initKinematics(robot_description,
                                          node->get_parameter("base_frame").as_string(),
                                          node->get_parameter("tool_frame").as_string()))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize kinematics.");
      return CallbackReturn::ERROR;
    }

    joint_names_ = robot_interface_->getJointNames();
    joint_limits_ = robot_interface_->getJointLimits();

    max_interpolation_velocity_ = node->get_parameter("max_velocity").as_double();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the Franka Cartesian velocity interface
    robot_interface_->assign_loaned_command(command_interfaces_);
    robot_interface_->assign_loaned_state(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_interface_->release_all_interfaces();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointPositionInterpolator::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (start && !done)
    {
      if (current_index < joint_trajectory.size())
      {
        robot_interface_->setCommand(joint_trajectory[current_index]);
        current_index++;
      }
      else
      {
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory complete.");
        done = true;
        start = false;
      }
    }
    return controller_interface::return_type::OK;
  }

  void JointPositionInterpolator::desiredPositionCallback(
      const extender_msgs::msg::JointPositionCommand::SharedPtr msg)
  {
    if (!done)
      return;

    target_positions_.clear();
    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      target_positions_[msg->joint_names[i]] = msg->desired_position[i];
    }

    computeTrajectory();
    current_index = 0;
  }

  void JointPositionInterpolator::computeTrajectory()
  {
    // get current position
    const std::vector<std::string> &st_names = robot_interface_->get_states_names();
    const std::vector<double> &st_values = robot_interface_->get_states_values();

    std::unordered_map<std::string, double> current_positions_map;
    for (size_t i = 0; i < st_names.size(); i++)
    {
      std::string sname = st_names[i];
      // Strip "/position" if necessary
      size_t slash_pos = sname.find('/');
      if (slash_pos != std::string::npos)
        sname = sname.substr(0, slash_pos);

      current_positions_map[sname] = st_values[i];
    }

    std::vector<InterpolateData> joint_plans;
    joint_plans.reserve(joint_names_.size());

    double max_duration = 0.0;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const std::string &jname = joint_names_[i];
      const auto &limits = joint_limits_[i];

      // Get Start Position
      if (current_positions_map.find(jname) == current_positions_map.end())
      {
        RCLCPP_WARN(get_node()->get_logger(), "Joint %s not found in state interface",
                    jname.c_str());
        joint_plans.push_back({0.0, 0.0, 1.0});
        continue;
      }
      double start = current_positions_map[jname];

      double target = start;
      if (target_positions_.find(jname) != target_positions_.end())
      {
        target = target_positions_[jname];
      }
      double diff = target - start;

      if (limits.jtype == robot_interfaces::JointType::CONTINUOUS)
      {
        diff = fmod(diff + M_PI, 2.0 * M_PI);
        if (diff < 0)
          diff += 2.0 * M_PI;
        diff -= M_PI;
      }

      double time_needed = std::abs(diff) / max_interpolation_velocity_;

      if (time_needed > max_duration)
      {
        max_duration = time_needed;
      }
      joint_plans.push_back({start, start + diff, max_interpolation_velocity_});
    }

    // Clear previous trajectories
    joint_trajectory.clear();
    joint_trajectory.resize(joint_names_.size());

    // default time step
    double dt = 0.01;

    size_t num_steps = (size_t)std::ceil(max_duration / dt);
    if (num_steps == 0)
      num_steps = 1;

    // Generate points
    for (size_t step = 0; step <= num_steps; ++step)
    {
      double alpha = (double)step / (double)num_steps;
      robot_interfaces::JointCommand jcommand;

      for (size_t j = 0; j < joint_names_.size(); ++j)
      {
        const auto &plan = joint_plans[j];
        double next_pos = plan.start_pos + (plan.end_pos - plan.start_pos) * alpha;
        jcommand.command.push_back(next_pos);
      }
      joint_trajectory.push_back(jcommand);
    }

    RCLCPP_INFO(get_node()->get_logger(), "Trajectory computed: %zu points over %.2f seconds.",
                num_steps, max_duration);
    start = true;
    done = false;
  }

} // namespace joint_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(joint_controllers::JointPositionInterpolator,
                       controller_interface::ControllerInterface)
