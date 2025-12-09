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
        get_node()->create_subscription<joint_position_interpolator::msg::JointPositionCommand>(
            "/joint_position_desired", 10,
            std::bind(&JointPositionInterpolator::desiredPositionCallback, this,
                      std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionInterpolator::on_configure(const rclcpp_lifecycle::State &)
  {
    std::string robot_type = get_node()->get_parameter("robot_type").as_string();
    robot_interface_ = robot_interfaces::create_robot_component(robot_type);
    if (!robot_interface_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create robot interface for type '%s'",
                   robot_type.c_str());
      return CallbackReturn::ERROR;
    }
    max_interpolation_velocity_ = get_node()->get_parameter("max_velocity").as_double();

    if (!parseLimits())
      return CallbackReturn::FAILURE;

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
    if (current_index >= joint_trajectory.size())
    {
      done = true;
      return controller_interface::return_type::OK;
    }
    else
    {
      const JointCommand jcommand = joint_trajectory[current_index];
      current_index++;

      robot_interface_->setCommand(jcommand);
      return controller_interface::return_type::OK;
    }
  }

  bool JointPositionInterpolator::parseLimits()
  {
    const std::vector<std::string> command_names = robot_interface_->get_commands_names();

    std::string urdf_xml;
    bool found_urdf = false;
    std_msgs::msg::String msg;

    auto temp_node = std::make_shared<rclcpp::Node>("temp_urdf_loader_node");

    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    qos_profile.reliable();
    auto timeout = std::chrono::seconds(2);

    RCLCPP_INFO(temp_node->get_logger(), "Waiting for /robot_description topic...");

    if (rclcpp::wait_for_message<std_msgs::msg::String>(msg, temp_node, "/robot_description",
                                                        timeout, qos_profile))
    {
      urdf_xml = msg.data;
      found_urdf = true;
      RCLCPP_INFO(temp_node->get_logger(), "Received URDF from topic!");
    }
    else
    {
      RCLCPP_WARN(temp_node->get_logger(), "Timed out waiting for /robot_description topic.");
      return false;
    }

    urdf::Model model;
    if (found_urdf)
    {
      if (!model.initString(urdf_xml))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF XML");
        return false; // invalidate if parse fails
      }
    }

    // 3. Set Limits
    joint_limits_.clear();
    for (const std::string &cname : command_names)
    {
      JointLimits jlimit;
      jlimit.has_position_limits = false;

      if (found_urdf)
      {
        // Handle name stripping if necessary (e.g. "joint1/position" -> "joint1")
        std::string simple_name = cname;
        size_t slash_pos = cname.find('/');
        if (slash_pos != std::string::npos)
        {
          simple_name = cname.substr(0, slash_pos);
          joint_names_.push_back(simple_name);
        }

        auto joint = model.getJoint(simple_name);
        if (joint)
        {
          switch (joint->type)
          {
          case urdf::Joint::REVOLUTE:
            jlimit.jtype = JointType::REVOLUTE;
            break;
          case urdf::Joint::CONTINUOUS:
            jlimit.jtype = JointType::CONTINUOUS;
            break;
          case urdf::Joint::PRISMATIC:
            jlimit.jtype = JointType::PRISMATIC;
            break;
          case urdf::Joint::FIXED:
            jlimit.jtype = JointType::FIXED;
            break;
          case urdf::Joint::FLOATING:
            jlimit.jtype = JointType::FLOATING;
            break;
          case urdf::Joint::PLANAR:
            jlimit.jtype = JointType::PLANAR;
            break;
          default:
            jlimit.jtype = JointType::UNKNOWN;
            break;
          }

          if (joint->limits)
          {
            jlimit.min_position = joint->limits->lower;
            jlimit.max_position = joint->limits->upper;
            jlimit.has_position_limits = true;
            jlimit.max_velocity = joint->limits->velocity;
            jlimit.has_velocity_limits = true;
          }
        }
      }
      joint_limits_.push_back(jlimit);
    }
    return true;
  }

  void JointPositionInterpolator::desiredPositionCallback(
      const joint_position_interpolator::msg::JointPositionCommand::SharedPtr msg)
  {
    if (!done)
      return;

    size_t index = 0;
    target_positions_.clear();
    for (const auto &jname : msg->joint_names)
    {

      const auto it = std::find(joint_names_.begin(), joint_names_.end(), jname);
      if (it == joint_names_.end())
      {
        index++;
        continue;
      }
      else
      {
        target_positions_.insert(std::pair(jname, msg->desired_position[index]));
        index++;
      }
    }
    current_index = 0;
    computeTrajectory();
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

      if (limits.jtype == JointType::CONTINUOUS)
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
  }

} // namespace joint_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(joint_controllers::JointPositionInterpolator,
                       controller_interface::ControllerInterface)
