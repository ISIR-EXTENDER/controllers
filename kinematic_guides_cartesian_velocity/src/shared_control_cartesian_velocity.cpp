#include "kinematic_guides_cartesian_velocity/shared_control_cartesian_velocity.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cartesian_velocity_controller
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SharedControlVelocityController::SharedControlVelocityController()
      : ControllerInterface(), latest_twist_(),
        current_orientation_(Eigen::Quaterniond::Identity()),
        current_position_(Eigen::Vector3d::Zero()),
        previous_cartesian_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_cartesian_angular_velocity_(Eigen::Vector3d::Zero()),
        previous_initial_filtered_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_initial_filtered_angular_velocity_(Eigen::Vector3d::Zero()),
        previous_filtered_linear_velocity_(Eigen::Vector3d::Zero()),
        previous_filtered_angular_velocity_(Eigen::Vector3d::Zero()),
        mode_(TeleopMode::Translation_Rotation), enable_shared_control_assistance_(false),
        enable_velocity_saturation_(false), enable_debug_publish_(false),
        initial_filter_cutoff_frequency_(0.0), final_filter_cutoff_frequency_(0.0),
        lpf_initialized_(false), max_linear_delta_(0.0), max_angular_delta_(0.0)
  {
  }

  SharedControlVelocityController::~SharedControlVelocityController()
  {
  }

  controller_interface::InterfaceConfiguration SharedControlVelocityController::
      command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // Use command interface names provided by the Franka Cartesian Velocity Interface.
    robot_vel_interface_->set_commands_names();

    config.names = robot_vel_interface_->get_commands_names();
    return config;
    return config;
  }

  controller_interface::InterfaceConfiguration SharedControlVelocityController::
      state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    robot_vel_interface_->set_states_names();

    config.names = robot_vel_interface_->get_states_names();
    return config;
  }

  CallbackReturn SharedControlVelocityController::on_init()
  {
    // Subscribe to custom TeleopCmd messages
    teleop_cmd_sub_ = get_node()->create_subscription<joystick_interface::msg::TeleopCmd>(
        "/teleop_cmd", 10,
        std::bind(&SharedControlVelocityController::teleopCmdCallback, this,
                  std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  SharedControlVelocityController::on_configure(const rclcpp_lifecycle::State &)
  {
    //------------------------------------------------------------------------------
    // Parameters declaration and loading
    //------------------------------------------------------------------------------

    // Check and declare parameters only if not already declared.
    // Flags to activate or desactive functions
    if (!get_node()->has_parameter("enable_shared_control_assistance"))
      get_node()->declare_parameter("enable_shared_control_assistance", true);
    if (!get_node()->has_parameter("enable_velocity_saturation"))
      get_node()->declare_parameter("enable_velocity_saturation", true);
    if (!get_node()->has_parameter("enable_debug_publish"))
      get_node()->declare_parameter("enable_debug_publish", false);

    // Low-pass filters parameters
    if (!get_node()->has_parameter("initial_filter_cutoff_frequency"))
      get_node()->declare_parameter("initial_filter_cutoff_frequency", 0.8);
    if (!get_node()->has_parameter("final_filter_cutoff_frequency"))
      get_node()->declare_parameter("final_filter_cutoff_frequency", 5.0);

    // Velocity saturation parameters
    if (!get_node()->has_parameter("max_linear_delta"))
      get_node()->declare_parameter("max_linear_delta", 0.007);
    if (!get_node()->has_parameter("max_angular_delta"))
      get_node()->declare_parameter("max_angular_delta", 0.014);

    // Shared control parameters
    if (!get_node()->has_parameter("gamma"))
      get_node()->declare_parameter("gamma", 3.0); // adimensional
    if (!get_node()->has_parameter("r2"))
      get_node()->declare_parameter("r2", 0.040); // m
    if (!get_node()->has_parameter("theta1_deg"))
      get_node()->declare_parameter("theta1_deg", 8.0); // deg
    if (!get_node()->has_parameter("theta2_deg"))
      get_node()->declare_parameter("theta2_deg", 4.0); // deg

    // Parameters for goal confidence update
    if (!get_node()->has_parameter("alpha_conf"))
      get_node()->declare_parameter("alpha_conf", 5.0); // s-1
    if (!get_node()->has_parameter("theta_l_deg"))
      get_node()->declare_parameter("theta_l_deg", 10.0); // deg
    if (!get_node()->has_parameter("r1"))
      get_node()->declare_parameter("r1", 0.08); // m
    if (!get_node()->has_parameter("v_j_max"))
      get_node()->declare_parameter("v_j_max", 0.04); // m/s

    // Read parameters from config file
    alpha_conf_ = get_node()->get_parameter("alpha_conf").as_double();
    theta_l_ = get_node()->get_parameter("theta_l_deg").as_double() * M_PI /
               180.0; // convert once because std::cos() expects radians
    r1_ = get_node()->get_parameter("r1").as_double();
    v_j_max_ = get_node()->get_parameter("v_j_max").as_double();
    gamma_ = get_node()->get_parameter("gamma").as_double();
    r2_ = get_node()->get_parameter("r2").as_double();
    theta1_ = get_node()->get_parameter("theta1_deg").as_double() * M_PI / 180.0;
    theta2_ = get_node()->get_parameter("theta2_deg").as_double() * M_PI / 180.0;
    enable_shared_control_assistance_ =
        get_node()->get_parameter("enable_shared_control_assistance").as_bool();
    enable_velocity_saturation_ = get_node()->get_parameter("enable_velocity_saturation").as_bool();
    enable_debug_publish_ = get_node()->get_parameter("enable_debug_publish").as_bool();
    initial_filter_cutoff_frequency_ =
        get_node()->get_parameter("initial_filter_cutoff_frequency").as_double();
    final_filter_cutoff_frequency_ =
        get_node()->get_parameter("final_filter_cutoff_frequency").as_double();
    max_linear_delta_ = get_node()->get_parameter("max_linear_delta").as_double();
    max_angular_delta_ = get_node()->get_parameter("max_angular_delta").as_double();

    /* -- Load multiple goals if available -- */
    // Not possible to load params as vector of vectors, need to declare a flat vector
    if (!get_node()->has_parameter("goals_poses"))
    {
      get_node()->declare_parameter<std::vector<double>>(
          "goals_poses", {0.605, 0.220, 0.278, 1.000, 0.003, 0.004, -0.001});
    }

    goals_poses_param_.clear();

    // Check goals are defined as a list of 7*N values, N being the number of goals
    auto goals_vector = get_node()->get_parameter("goals_poses").as_double_array();
    if (goals_vector.size() % 7 != 0)
    {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "'goals_poses' must be 7*N values: [x y z qx qy qz qw] × N. Got %zu.",
                   goals_vector.size());
      return CallbackReturn::ERROR;
    }

    // Parse and push back goals
    const size_t N = goals_vector.size() / 7;
    goals_poses_param_.reserve(N);

    for (size_t k = 0; k < N; ++k)
    {
      std::array<double, 7> goal{
          goals_vector[7 * k + 0], goals_vector[7 * k + 1], goals_vector[7 * k + 2], // x y z
          goals_vector[7 * k + 3], goals_vector[7 * k + 4], goals_vector[7 * k + 5], // qx qy qz
          goals_vector[7 * k + 6]                                                    // qw
      };
      goals_poses_param_.push_back(goal);
    }

    // Log goals in console
    RCLCPP_INFO(get_node()->get_logger(), "Loaded %zu 6D goal(s).", N);
    for (size_t k = 0; k < N; ++k)
    {
      const auto &goal = goals_poses_param_[k];
      const double qnorm =
          std::sqrt(goal[3] * goal[3] + goal[4] * goal[4] + goal[5] * goal[5] + goal[6] * goal[6]);
      RCLCPP_INFO(get_node()->get_logger(),
                  "  G%zu: pos=[%.3f, %.3f, %.3f], q=[w=%.3f, x=%.3f, y=%.3f, z=%.3f] (||q||=%.4f)",
                  k + 1, goal[0], goal[1], goal[2], goal[6], goal[3], goal[4], goal[5], qnorm);
      if (qnorm < 1e-9)
      {
        RCLCPP_WARN(get_node()->get_logger(), "    G%zu quaternion norm ≈ 0 — will use identity.",
                    k + 1);
      }
      else if (std::abs(qnorm - 1.0) > 1e-2)
      {
        RCLCPP_WARN(get_node()->get_logger(), "    G%zu quaternion not unit — will be normalized.",
                    k + 1);
      }
    }

    /* -- Reach feedback params -- */
    if (!get_node()->has_parameter("eps_t_reach_m"))
      get_node()->declare_parameter("eps_t_reach_m", 0.01); // 1 cm
    if (!get_node()->has_parameter("eps_r_reach_deg"))
      get_node()->declare_parameter("eps_r_reach_deg", 5.0); // 5 deg
    if (!get_node()->has_parameter("reach_dwell_ms"))
      get_node()->declare_parameter("reach_dwell_ms", 300); // 300 ms

    eps_t_reach_ = get_node()->get_parameter("eps_t_reach_m").as_double();
    eps_r_reach_rad_ = get_node()->get_parameter("eps_r_reach_deg").as_double() * M_PI / 180.0;
    dwell_ms_ = get_node()->get_parameter("reach_dwell_ms").as_int();

    // Known goals (can be different from goals_poses_param)
    if (!get_node()->has_parameter("known_goals_poses"))
    {
      get_node()->declare_parameter<std::vector<double>>("known_goals_poses",
                                                         std::vector<double>{});
    }
    if (!get_node()->has_parameter("known_goals_labels"))
    {
      get_node()->declare_parameter<std::vector<std::string>>("known_goals_labels",
                                                              std::vector<std::string>{});
    }

    known_goals_poses_param_.clear();
    known_goals_labels_.clear();

    const auto known_goals_vector =
        get_node()->get_parameter("known_goals_poses").as_double_array();
    const auto kg_labels = get_node()->get_parameter("known_goals_labels").as_string_array();
    known_goals_labels_.assign(kg_labels.begin(), kg_labels.end());

    // Must be 7*N
    if (known_goals_vector.size() % 7 != 0)
    {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "'known_goals_poses' must be 7*N values: [x y z qx qy qz qw] × N. Got %zu.",
                   known_goals_vector.size());
      return CallbackReturn::ERROR;
    }

    const size_t K = known_goals_vector.size() / 7;
    known_goals_poses_param_.reserve(K);

    for (size_t k = 0; k < K; ++k)
    {
      std::array<double, 7> goal{
          known_goals_vector[7 * k + 0], known_goals_vector[7 * k + 1],
          known_goals_vector[7 * k + 2], // x y z
          known_goals_vector[7 * k + 3], known_goals_vector[7 * k + 4],
          known_goals_vector[7 * k + 5], // qx qy qz
          known_goals_vector[7 * k + 6]  // qw
      };
      known_goals_poses_param_.push_back(goal);
    }

    // Log known goals in console
    RCLCPP_INFO(get_node()->get_logger(), "Loaded %zu known 6D goal(s).", K);
    for (size_t k = 0; k < K; ++k)
    {
      const auto &known_goal = known_goals_poses_param_[k];
      const double qnorm = std::sqrt(known_goal[3] * known_goal[3] + known_goal[4] * known_goal[4] +
                                     known_goal[5] * known_goal[5] + known_goal[6] * known_goal[6]);
      const bool has_label = (k < known_goals_labels_.size() && !known_goals_labels_[k].empty());
      const char *label = has_label ? known_goals_labels_[k].c_str() : nullptr;

      if (label)
      {
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  KG%zu '%s': pos=[%.3f, %.3f, %.3f], q=[w=%.3f, x=%.3f, y=%.3f, z=%.3f] (||q||=%.4f)",
            k + 1, label, known_goal[0], known_goal[1], known_goal[2], known_goal[6], known_goal[3],
            known_goal[4], known_goal[5], qnorm);
      }
      else
      {
        RCLCPP_INFO(
            get_node()->get_logger(),
            "  KG%zu: pos=[%.3f, %.3f, %.3f], q=[w=%.3f, x=%.3f, y=%.3f, z=%.3f] (||q||=%.4f)",
            k + 1, known_goal[0], known_goal[1], known_goal[2], known_goal[6], known_goal[3],
            known_goal[4], known_goal[5], qnorm);
      }

      if (qnorm < 1e-9)
      {
        RCLCPP_WARN(get_node()->get_logger(),
                    "    KG%zu quaternion norm ≈ 0 — will use identity at check time.", k + 1);
      }
      else if (std::abs(qnorm - 1.0) > 1e-2)
      {
        RCLCPP_WARN(get_node()->get_logger(),
                    "    KG%zu quaternion not unit — will be normalized at check time.", k + 1);
      }
    }

    //------------------------------------------------------------------------------
    // Debug publishers
    //------------------------------------------------------------------------------
    latest_twist_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::Twist>("/debug/latest_twist", 10);
    initial_filtered_linear_velocity_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::Vector3>(
            "/debug/initial_filtered_linear_velocity", 10);
    initial_filtered_angular_velocity_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::Vector3>(
            "/debug/initial_filtered_angular_velocity", 10);
    assisted_linear_velocity_pub_ = get_node()->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/assisted_linear_velocity", 10);
    assisted_angular_velocity_pub_ = get_node()->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/assisted_angular_velocity", 10);
    filtered_linear_velocity_pub_ = get_node()->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/final_filtered_linear_velocity", 10);
    filtered_angular_velocity_pub_ = get_node()->create_publisher<geometry_msgs::msg::Vector3>(
        "/debug/final_filtered_angular_velocity", 10);
    conf_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/debug/goal_confidences", 10); // goal confidences
    soft_goal_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/soft_goal", 10);
    agnostic_goal_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/debug/agnostic_goal", 10);
    beep_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>("/feedback/beep_trigger", 10);
    goal_reached_pub_ =
        get_node()->create_publisher<std_msgs::msg::Int32>("/debug/goal_reached", 10);
    mode_pub_ = get_node()->create_publisher<std_msgs::msg::Int32>("/debug/mode", 10);

    //------------------------------------------------------------------------------
    // Logging
    //------------------------------------------------------------------------------
    RCLCPP_INFO(get_node()->get_logger(), "Active Functions:");
    RCLCPP_INFO(get_node()->get_logger(), "  Shared control: %s",
                enable_shared_control_assistance_ ? "ON" : "OFF");
    RCLCPP_INFO(get_node()->get_logger(), "  Velocity saturation: %s",
                enable_velocity_saturation_ ? "ON" : "OFF");
    RCLCPP_INFO(get_node()->get_logger(), "  Debug publish:       %s",
                enable_debug_publish_ ? "ON" : "OFF");

    RCLCPP_INFO(get_node()->get_logger(), "Filtering and Saturation Parameters");
    RCLCPP_INFO(get_node()->get_logger(), "  Initial filter frequency: %.1f",
                initial_filter_cutoff_frequency_);
    RCLCPP_INFO(get_node()->get_logger(), "  Final filter frequency: %.1f",
                final_filter_cutoff_frequency_);
    RCLCPP_INFO(get_node()->get_logger(), "  max_linear_delta: %.4f", max_linear_delta_);
    RCLCPP_INFO(get_node()->get_logger(), "  max_angular_delta: %.4f", max_angular_delta_);

    // ---------- Shared Control parameters ----------
    RCLCPP_INFO(get_node()->get_logger(), "Shared Control Parameters:");
    RCLCPP_INFO(get_node()->get_logger(), "  alpha_conf:          %.1f  [s^-1]", alpha_conf_);
    RCLCPP_INFO(get_node()->get_logger(), "  theta_l:             %.1f  [deg]",
                get_node()->get_parameter("theta_l_deg").as_double());
    RCLCPP_INFO(get_node()->get_logger(), "  gamma:              %.1f", gamma_);
    RCLCPP_INFO(get_node()->get_logger(), "  r1:                 %.3f  [m]", r1_);
    RCLCPP_INFO(get_node()->get_logger(), "  r2:                 %.3f  [m]", r2_);
    RCLCPP_INFO(get_node()->get_logger(), "  v_j_max:            %.3f  [m/s]", v_j_max_);
    RCLCPP_INFO(get_node()->get_logger(), "Reach Feedback:");
    RCLCPP_INFO(get_node()->get_logger(),
                "  eps_t_reach: %.3f m, eps_r_reach: %.1f deg, dwell: %d ms", eps_t_reach_,
                get_node()->get_parameter("eps_r_reach_deg").as_double(), dwell_ms_);

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

  CallbackReturn SharedControlVelocityController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Assign the loaned command interfaces to the Franka Cartesian Velocity Interface.
    robot_vel_interface_->assign_loaned_command(command_interfaces_);
    robot_vel_interface_->assign_loaned_state(state_interfaces_);

    // Reset filtering and saturation
    previous_initial_filtered_linear_velocity_.setZero();
    previous_initial_filtered_angular_velocity_.setZero();
    previous_cartesian_linear_velocity_.setZero();
    previous_cartesian_angular_velocity_.setZero();
    previous_filtered_linear_velocity_.setZero();
    previous_filtered_angular_velocity_.setZero();

    lpf_initialized_ = false;
    latest_twist_ = geometry_msgs::msg::Twist{}; // zeros
    last_reached_goal_idx_ = -1;
    beeped_for_this_visit_ = false;

    // Set current pose and initialise Goals
    robot_interfaces::CartesianPosition temp_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();
    current_position_ = temp_pose.translation;
    current_orientation_ = temp_pose.quaternion;

    initialiseGoals();

    if (mode_pub_)
    {
      std_msgs::msg::Int32 mode;
      mode.data = static_cast<int>(mode_);
      mode_pub_->publish(mode);
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SharedControlVelocityController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    robot_vel_interface_->release_all_interfaces();
    return CallbackReturn::SUCCESS;
  }

  /* -- Update function -- */
  controller_interface::return_type SharedControlVelocityController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    /* -- Update state variables --*/
    // Get time
    const rclcpp::Time now = get_node()->now();
    // Get controller period
    const double dt_sec = period.seconds();
    // Get current EE pose.
    robot_interfaces::CartesianPosition temp_pose =
        robot_vel_interface_->getCurrentEndEffectorPose();
    current_position_ = temp_pose.translation;
    current_orientation_ = temp_pose.quaternion;
    // Sync agnostic goal: G0 pose = current EE pose
    if (!goals_.empty())
    {
      goals_[0].x = current_position_;
      goals_[0].q = current_orientation_;
    }

    /* -- Raw velocities from fr3_teleop twist -- */
    const Eigen::Vector3d raw_linear_velocity(latest_twist_.linear.x, latest_twist_.linear.y,
                                              latest_twist_.linear.z);

    const Eigen::Vector3d raw_angular_velocity(latest_twist_.angular.x, latest_twist_.angular.y,
                                               latest_twist_.angular.z);

    /* -- Apply a low-pass filter to the latest twist command -- */
    // Compute alpha from desired cutoff frequency: alpha = T/(T+tau)
    double alpha_initial_filter = 1.0; // default: passthrough if fc <= 0

    if (initial_filter_cutoff_frequency_ > 0.0)
    {
      const double tau = 1.0 / (2.0 * M_PI * initial_filter_cutoff_frequency_);
      if (dt_sec > 0.0)
      {
        alpha_initial_filter = std::clamp(dt_sec / (dt_sec + tau), 1e-9, 1.0 - 1e-9);
      }
    } // else alpha stays 1 so there's no filtering

    if (!lpf_initialized_)
    {
      previous_initial_filtered_linear_velocity_ = raw_linear_velocity;
      previous_initial_filtered_angular_velocity_ = raw_angular_velocity;
      previous_cartesian_linear_velocity_ = Eigen::Vector3d::Zero();
      previous_cartesian_angular_velocity_ = Eigen::Vector3d::Zero();
      lpf_initialized_ = true;
    }

    const Eigen::Vector3d initial_filtered_linear_velocity = applyLowPassFilterVector(
        raw_linear_velocity, previous_initial_filtered_linear_velocity_, alpha_initial_filter);

    const Eigen::Vector3d initial_filtered_angular_velocity = applyLowPassFilterVector(
        raw_angular_velocity, previous_initial_filtered_angular_velocity_, alpha_initial_filter);

    // Update memory
    previous_initial_filtered_linear_velocity_ = initial_filtered_linear_velocity;
    previous_initial_filtered_angular_velocity_ = initial_filtered_angular_velocity;

    // Rotation from End-Effector frame to Base frame
    const Eigen::Matrix3d R_BE = current_orientation_.toRotationMatrix();
    // Convert joystick angular velocity (EE frame) to Base frame
    const Eigen::Vector3d initial_filtered_angular_velocity_base =
        R_BE * initial_filtered_angular_velocity;

    /* -- Update Goals confidences only in MODE T (Translation_Rotation) -- */
    if (mode_ == TeleopMode::Translation_Rotation)
    {
      updateConfidences(dt_sec, initial_filtered_linear_velocity);
    }

    /* -- Compute goal selection based on confidence index -- */
    Goal soft_goal = computeSoftGoal();

    /* -- Check goals -- */
    // Feedback check of known goals
    checkKnownGoalsAndBeep(now);

    /* -- Shared-control -- */
    Eigen::Vector3d cartesian_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d cartesian_angular_velocity = Eigen::Vector3d::Zero();

    if (enable_shared_control_assistance_)
    {
      // -- Shared control assistance -- //
      switch (mode_)
      {
      case TeleopMode::Translation_Rotation: { // MODE T

        auto [shaped_velocity, shaped_omega] = applySharedControlModeT(
            soft_goal, current_position_, current_orientation_, initial_filtered_linear_velocity);

        cartesian_linear_velocity = shaped_velocity;
        cartesian_angular_velocity = shaped_omega;
        break;
      }
      case TeleopMode::Rotation: { // MODE W
        auto [shaped_velocity, shaped_omega] =
            applySharedControlModeW(soft_goal, current_position_, current_orientation_,
                                    initial_filtered_angular_velocity_base);

        cartesian_linear_velocity = shaped_velocity;
        cartesian_angular_velocity = shaped_omega;
        break;
      }
      default:
        cartesian_linear_velocity.setZero();
        cartesian_angular_velocity.setZero();
        break;
      }
    }
    else
    {
      // --- Baseline: agnostic control mode ---
      switch (mode_)
      {
      case TeleopMode::Translation_Rotation: {
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        // Compute baseline angular velocity to keep wrist aligned
        cartesian_angular_velocity =
            computeBaselineAngularVelocity(current_position_, cartesian_linear_velocity);
        break;
      }
      case TeleopMode::Rotation: {
        cartesian_linear_velocity.setZero();
        // User commands rotation in the end-effector frame, command is sent to robot in base frame
        cartesian_angular_velocity = initial_filtered_angular_velocity_base;
        break;
      }
      case TeleopMode::Translation: {
        // Used for debug in advanced mode: allow_full_mode = true in fr3_teleop
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        cartesian_angular_velocity.setZero();
        break;
      }
      // TODO: legacy, to remove
      case TeleopMode::Both: { // not like Mode W, just to use spacenav as full 6D joystick
        // Used for debug in advanced mode: allow_full_mode = true in fr3_teleop
        cartesian_linear_velocity = initial_filtered_linear_velocity;
        cartesian_angular_velocity = initial_filtered_angular_velocity;
        break;
      }
      default:
        cartesian_linear_velocity.setZero();
        cartesian_angular_velocity.setZero();
        break;
      }
    }

    /* -- Cartesian velocity saturation -- */
    // Limit the velocity change between cycles to avoid discontinuties.
    Eigen::Vector3d saturated_cartesian_linear_velocity = cartesian_linear_velocity;
    Eigen::Vector3d saturated_cartesian_angular_velocity = cartesian_angular_velocity;

    if (enable_velocity_saturation_)
    {
      std::tie(saturated_cartesian_linear_velocity, saturated_cartesian_angular_velocity) =
          applyVelocitySaturation(cartesian_linear_velocity, previous_cartesian_linear_velocity_,
                                  max_linear_delta_, cartesian_angular_velocity,
                                  previous_cartesian_angular_velocity_, max_angular_delta_);

      // Update memory
      previous_cartesian_linear_velocity_ = saturated_cartesian_linear_velocity;
      previous_cartesian_angular_velocity_ = saturated_cartesian_angular_velocity;
    }
    else
    { // Avoid spikes when turning saturation on
      previous_cartesian_linear_velocity_ = cartesian_linear_velocity;
      previous_cartesian_angular_velocity_ = cartesian_angular_velocity;
    }
    // -----

    /* -- Post-saturation low-pass filter (5 Hz default) -- */
    double alpha_final_filter = 1.0; // default: passthrough if fc <= 0
    if (final_filter_cutoff_frequency_ > 0.0)
    {
      const double tau_final_filter = 1.0 / (2.0 * M_PI * final_filter_cutoff_frequency_);
      if (dt_sec > 0.0)
      {
        alpha_final_filter = std::clamp(dt_sec / (dt_sec + tau_final_filter), 1e-9, 1.0 - 1e-9);
      }
    }

    const Eigen::Vector3d filtered_linear_velocity =
        applyLowPassFilterVector(saturated_cartesian_linear_velocity,
                                 previous_filtered_linear_velocity_, alpha_final_filter);

    const Eigen::Vector3d filtered_angular_velocity =
        applyLowPassFilterVector(saturated_cartesian_angular_velocity,
                                 previous_filtered_angular_velocity_, alpha_final_filter);

    // Update memory
    previous_filtered_linear_velocity_ = filtered_linear_velocity;
    previous_filtered_angular_velocity_ = filtered_angular_velocity;

    /* -- Publish data -- */
    if (enable_debug_publish_)
    {
      publishDebugData(latest_twist_, initial_filtered_linear_velocity,
                       initial_filtered_angular_velocity, cartesian_linear_velocity,
                       cartesian_angular_velocity, filtered_linear_velocity,
                       filtered_angular_velocity, soft_goal);
    }

    robot_interfaces::CartesianVelocity vel_cmd;
    vel_cmd.linear = filtered_linear_velocity;
    vel_cmd.angular = filtered_angular_velocity;
    /* -- Send cartesian velocity commands using Franka Cartesian Velocity interface -- */
    if (robot_vel_interface_->setCommand(vel_cmd))
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }

  } // update()

  // -----------------------------------------------------------------------------
  // teleopCmdCallback()
  // -----------------------------------------------------------------------------
  // Receives joystick commands from the /teleop_cmd topic and updates the internal
  // teleoperation state.
  //
  // The incoming message contains:
  //   • msg->twist : geometry_msgs::Twist with user linear/angular velocity commands
  //   • msg->mode  : integer enum corresponding to a TeleopMode (e.g. Translation,
  //                  Rotation, or Translation_Rotation)
  //
  // The callback updates the latest twist command and switches control mode if it
  // differs from the current one. In all cases, the current mode is published on
  // the /debug/mode topic for visualization and logging.
  // -----------------------------------------------------------------------------
  void SharedControlVelocityController::teleopCmdCallback(
      const joystick_interface::msg::TeleopCmd::SharedPtr msg)
  {
    latest_twist_ = msg->twist;
    const TeleopMode new_mode = static_cast<TeleopMode>(msg->mode);

    if (new_mode != mode_)
    {
      mode_ = new_mode;
    }

    // Always publish current mode (either new or unchanged)
    if (mode_pub_)
    {
      std_msgs::msg::Int32 mode_msg;
      mode_msg.data = static_cast<int>(mode_);
      mode_pub_->publish(mode_msg);
    }
  }

  // -----------------------------------------------------------------------------
  // computeDistanceAndDirection()
  // -----------------------------------------------------------------------------
  // Computes the Euclidean distance ε_d and unit direction û from the current
  // end-effector position x_E to the goal position x_G.
  //
  // Returns a pair {ε_d, û}, where:
  //   ε_d = ‖x_G − x_E‖
  //   û   = (x_G − x_E) / ε_d   if ε_d > 0
  //       = [1, 0, 0]ᵀ          otherwise (default axis)
  // -----------------------------------------------------------------------------
  std::pair<double, Eigen::Vector3d> SharedControlVelocityController::computeDistanceAndDirection(
      const Eigen::Vector3d &goal_position, const Eigen::Vector3d &current_position) const
  {
    const Eigen::Vector3d direction = goal_position - current_position;
    const double distance = direction.norm();

    Eigen::Vector3d unit_axis_u = Eigen::Vector3d::UnitX();
    if (distance > 1e-9)
    {
      unit_axis_u = direction / distance;
    }

    return {distance, unit_axis_u};
  }

  // -----------------------------------------------------------------------------
  // computeRotationError()
  // -----------------------------------------------------------------------------
  // Computes the minimal 3D rotation required to align the current end-effector
  // orientation (q_current) with the desired goal orientation (q_goal).
  //
  // The relative rotation quaternion is defined as:
  //     q_err = q_goal * q_current.conjugate()
  // which represents the rotation that transforms the current frame to the goal
  // frame.
  //
  // Because unit quaternions q and -q represent the same physical rotation,
  // the scalar part w is flipped to ensure w ≥ 0 so that the shortest rotation
  // path is always used to avoid discontinuities.
  //
  // The Eigen::AngleAxisd representation is then used to extract:
  //   • angle  θ  = angle_axis.angle()       total rotation angle ε_r ∈ [0, π]
  //   • axis   ŵ  = angle_axis.axis()        unit rotation axis
  //   • e_R = θ * ŵ                          SO(3) logarithmic map (rotation vector)
  //
  // Returns a RotationError struct { angle, axis, rotation_vector }.
  // -----------------------------------------------------------------------------
  auto SharedControlVelocityController::computeRotationError(
      const Eigen::Quaterniond &q_goal, const Eigen::Quaterniond &q_current) const -> RotationError
  {
    // Relative rotation (from current to goal)
    Eigen::Quaterniond q_err = q_goal * q_current.conjugate();

    // Shortest path: ensure w >= 0 (q and -q represent same rotation)
    if (q_err.w() < 0.0)
      q_err.coeffs() *= -1.0;

    // Extract angle and axis using Eigen
    Eigen::AngleAxisd angle_axis(q_err);
    const double theta = angle_axis.angle();
    const Eigen::Vector3d axis = angle_axis.axis();

    // SO(3) logarithmic map: e_R = θ * ŵ
    const Eigen::Vector3d rotation_vector = theta * axis;

    return RotationError{theta, axis, rotation_vector};
  }

  // -----------------------------------------------------------------------------
  // applyLowPassFilterVector()
  // -----------------------------------------------------------------------------
  // Applies a first-order discrete low-pass filter to a 3D input vector.
  //
  // Implements the standard exponential smoothing form:
  //     y[n] = α * x[n] + (1 - α) * y[n-1]
  // where:
  //   • x[n]       current (raw) input vector
  //   • y[n-1]     previously filtered output
  //   • α ∈ [0,1]  filter gain (computed as T / (T + τ))
  //
  // Interpretation:
  //   - α = 1  minimal filtering (fast response)
  //   - α = 0  strong filtering (slow response)
  //   - τ      time constant of the filter (s)
  //   - T      sampling period (s)
  //
  // This filter is applied component-wise on the input Eigen::Vector3d.
  // It is used to smooth measured or commanded Cartesian velocities.
  // -----------------------------------------------------------------------------
  Eigen::Vector3d SharedControlVelocityController::applyLowPassFilterVector(
      const Eigen::Vector3d &input, const Eigen::Vector3d &previous, double alpha)
  {
    return alpha * input + (1.0 - alpha) * previous;
  }

  // -----------------------------------------------------------------------------
  // initialiseGoals()
  // -----------------------------------------------------------------------------
  // Initializes the internal list of 6D goals used by the shared-control system.
  //
  // The function constructs the following goal set:
  //
  //   • G₀ — Agnostic goal (current EE pose, confidence = 1.0)
  //     Acts as a fallback and ensures continuity when no specific goal dominates.
  //
  //   • G₁…Gₙ — User-defined 6D goals loaded from the 'goals_poses' parameter,
  //     each defined as [x, y, z, qx, qy, qz, qw].
  //
  // Each goal is stored as a Goal struct { x, q, c }, where:
  //   • x : 3D Cartesian position [m]
  //   • q : normalized orientation quaternion (Eigen convention w,x,y,z)
  //   • c : confidence (initialized to 1.0 for G₀, 0.0 for others)
  //
  // Quaternions are automatically normalized; degenerate inputs (‖q‖ ≈ 0) are
  // replaced with the identity rotation to ensure stability.
  //
  // Logs all initialized goals and their parameters for verification.
  //
  // -----------------------------------------------------------------------------
  void SharedControlVelocityController::initialiseGoals()
  {
    goals_.clear();

    // ---------------------------------------------------------------------
    // [G0] AGNOSTIC GOAL – current EE pose with confidence = 1
    //     (guarantees continuity / fallback)
    // ---------------------------------------------------------------------
    goals_.push_back(Goal{current_position_, current_orientation_, 1.0});
    RCLCPP_INFO(get_node()->get_logger(),
                "G0 (agnostic): pos=[%.3f, %.3f, %.3f], q=[w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                current_position_.x(), current_position_.y(), current_position_.z(),
                current_orientation_.w(), current_orientation_.x(), current_orientation_.y(),
                current_orientation_.z());

    // ---------------------------------------------------------------------
    // [G1…Gn] USER-DEFINED GOALS
    //  • strictly 6D
    // ---------------------------------------------------------------------
    const size_t N = goals_poses_param_.size();
    for (size_t k = 0; k < N; ++k)
    {
      const auto &goal = goals_poses_param_[k]; // [x y z qx qy qz qw]

      const Eigen::Vector3d position(goal[0], goal[1], goal[2]);
      Eigen::Quaterniond orientation_quaternion(goal[6], goal[3], goal[4],
                                                goal[5]); // Eigen (w,x,y,z)

      const double n = orientation_quaternion.norm();
      if (n < 1e-9)
      {
        RCLCPP_WARN(get_node()->get_logger(), "G%zu quaternion norm ≈ 0 → using identity.", k + 1);
        orientation_quaternion = Eigen::Quaterniond::Identity();
      }
      else
      {
        orientation_quaternion.normalize();
      }

      goals_.push_back(Goal{position, orientation_quaternion, 0.0});
      RCLCPP_INFO(get_node()->get_logger(),
                  "G%zu: pos=[%.3f, %.3f, %.3f], q=[w=%.3f, x=%.3f, y=%.3f, z=%.3f]", k + 1,
                  position.x(), position.y(), position.z(), orientation_quaternion.w(),
                  orientation_quaternion.x(), orientation_quaternion.y(),
                  orientation_quaternion.z());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Initialized %zu 6D user goal(s) + G0 (agnostic).", N);
  }

  // -----------------------------------------------------------------------------
  // updateConfidences()
  // -----------------------------------------------------------------------------
  // Implements the confidence integration mechanism from Algorithm 1
  // to estimate which goal the user is currently moving toward.
  //
  // Each real goal Gᵢ (i≥1) has a confidence cᵢ ∈ [0,1] updated as:
  //
  //   ẋcᵢ = α_conf ⋅ (‖v_E‖ / v_Jmax) ⋅ ((cosφ_W − cosθ_l) / (1 − cosθ_l))
  //
  // where:
  //   • cosφ_W = ( (W uᵢ)ᵀ (W v_E) ) / (‖W uᵢ‖ ‖W v_E‖)     weighted directional cosine
  //   • W = diag(1, 1, z_scale)                             anisotropic weighting (to reduce
  //   Z-sensitivity) • θ_l  = acceptance cone half-angle • α_conf = integration gain [s⁻¹]
  //
  // Integration occurs only if:
  //   • the user’s motion speed ≥ v₁ (20 % threshold),
  //   • and the end-effector is outside all freeze spheres (r_freeze = min(r₁, r₂)).
  //
  // Otherwise, confidence updates are frozen.
  //
  // The agnostic goal G₀ keeps c₀ = max(0, 1 − Σ cᵢ), ensuring total confidence sums to 1.
  //
  // Notes:
  //   • θ-normalization ensures α_conf and θ_l are numerically decoupled.
  //   • Weighted cosine uses explicit z-scaling on both uᵢ and v_E (z_scale = 0.2).
  // -----------------------------------------------------------------------------
  void SharedControlVelocityController::updateConfidences(double dt, const Eigen::Vector3d &vE)
  {
    const double vJmax = v_j_max_;              // measure once
    const double z_scale = 0.2;                 // divide z sensitivity by 5
    const double r_freeze = std::min(r1_, r2_); // define a freeze sphere as the smallest one

    // --- Unit-speed and v1 threshold ---
    const double speed_scale = std::min(1.0, vE.norm() / vJmax); // ∈ [0,1]
    const double v1 = 0.20;                                      // 20% threshold

    // --- θ-normalization to decouple α_conf and θ_l ---
    const double cos_th = std::cos(theta_l_);
    double denom_theta = 1.0 - cos_th;

    if (denom_theta <= 1e-6)
    {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[updateConfidences] theta_l too small (θ=%.6f rad). "
                  "Clamping 1 - cos(θ) to 1e-6 for normalization.",
                  theta_l_);
      denom_theta = 1e-6;
    }

    // --- Step 1: Freeze detection ---
    bool inside_any_r_freeze = false;
    size_t freeze_idx = 0; // goal index (G1..Gn) responsible for the freeze
    for (size_t i = 1; i < goals_.size(); ++i)
    { // skip G0 (agnostic)
      const auto distance_direction = computeDistanceAndDirection(goals_[i].x, current_position_);
      const double epsilon_d_i = distance_direction.first;
      if (epsilon_d_i < r_freeze)
      {
        inside_any_r_freeze = true;
        break; // no need to check further
      }
    }

    // --- Step 2: Confidence update ---
    if (!inside_any_r_freeze && speed_scale >= v1)
    {
      // Outside all r_freeze spheres: integrate confidences for all real goals
      for (size_t i = 1; i < goals_.size(); ++i)
      { // G2…Gn

        const auto [epsilon_d, unit_axis] =
            computeDistanceAndDirection(goals_[i].x, current_position_);

        // Note: using variables names close to equations notation for clarity.
        // Weighted direction
        Eigen::Vector3d u_W = unit_axis; // weighted unit axis
        u_W.z() *= z_scale;              // scale in z
        const double norm_Wu = u_W.norm();

        // Weighted velocity
        Eigen::Vector3d W_v = vE; // weighted velocity
        W_v.z() *= z_scale;       // scale in z
        const double norm_Wv = W_v.norm();

        // Directional cosine in weighted space
        double cos_phi_w = 0.0; // cosine of the angle in the weigthed space
        if (norm_Wv > 0.0 && norm_Wu > 0.0)
        { // to avoid division by zero
          cos_phi_w = u_W.dot(W_v) / (norm_Wu * norm_Wv);
          cos_phi_w = std::clamp(cos_phi_w, -1.0, 1.0); // to protect against numerical overshoots
        }

        // θ-normalized cone term
        const double normalized_cone = (cos_phi_w - cos_th) / denom_theta;

        // Final confidence derivative
        // Eq. (8): cdot = α_conf * (‖v‖/vJmax) * (cos_phi_w − cos θ_l) / (1 − cos θ_l)
        const double cdot = alpha_conf_ * speed_scale * normalized_cone;

        goals_[i].c = std::clamp(goals_[i].c + cdot * dt, 0.0, 1.0); // cGi[0,1]
      }
    }
    else
    {
      // Debug: Freeze update info
      /*
      const double ed = (goals_[freeze_idx].x - current_position_).norm();
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[updateConfidences] FREEZE: inside r1 of goal G%zu — ed=%.3f . "
        "No confidences updated this cycle.",
        freeze_idx, ed);
        */
    }

    // Agnostic confidence
    double sum = 0.0;
    for (size_t i = 1; i < goals_.size(); ++i)
      sum += goals_[i].c;
    goals_[0].c = std::max(0.0, 1.0 - sum);
  }

  // -----------------------------------------------------------------------------
  // computeSoftGoal()
  // -----------------------------------------------------------------------------
  // Weighted soft goal synthesis (Eq. 4):
  //   x_G = (∑ c_i x_i) / (∑ c_i)
  //   r_G = argmax_eig  of  A = ∑ c_i (q_i q_iᵀ)  (Markley quaternion average)
  // -----------------------------------------------------------------------------
  Goal SharedControlVelocityController::computeSoftGoal() const
  {
    double confidence_sum = 0.0;
    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d xG = Eigen::Vector3d::Zero();
    Eigen::Matrix4d markley_matrixQ = Eigen::Matrix4d::Zero(); // Markley accumulator

    for (const auto &goal : goals_)
    {
      if (goal.c <= 0.0)
        continue;

      // Translation accumulator
      weighted_sum += goal.c * goal.x;
      confidence_sum += goal.c;

      // Rotation accumulator
      const Eigen::Vector4d qv(goal.q.w(), goal.q.x(), goal.q.y(), goal.q.z());
      markley_matrixQ += goal.c * (qv * qv.transpose());
    }

    // Weigthed average Translation
    if (confidence_sum > 0.0)
    {
      xG = weighted_sum / confidence_sum;
    } // else: keep zero; shouldn't happen because G0 has c=1

    // SelfAdjointEigenSolver sorts eigenvalues ascending: last col is largest eigenvector
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(markley_matrixQ);
    if (eigen_solver.info() != Eigen::Success)
    {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "[computeSoftGoal] Eigen decomposition failed (info=%d).",
                   static_cast<int>(eigen_solver.info()));
      std::terminate();
    }

    // Weighted average Rotation
    Eigen::Vector4d average_quaternion =
        eigen_solver.eigenvectors().col(3); // eigenvector with largest eigenvalue
    Eigen::Quaterniond rG(average_quaternion(0), average_quaternion(1), average_quaternion(2),
                          average_quaternion(3));
    rG.normalize(); // ensure unit quaternion

    return {xG, rG, 1.0};
  }

  // -----------------------------------------------------------------------------
  // computeBaselineAngularVelocity()
  // -----------------------------------------------------------------------------
  // Computes the baseline angular velocity ω_{E,u,T} used in MODE T
  // to keep the wrist aligned with the mobile (base) frame.
  //
  // Implements the geometric relationship:
  //     ω_z = (x_E * v_y - y_E * v_x) / (x_E² + y_E²)
  //
  // This expression represents the instantaneous angular rate needed
  // to maintain constant wrist orientation relative to the mobile frame
  // during translational motion. When the end-effector moves purely
  // along the base frame axes, ω_z = 0.
  // -----------------------------------------------------------------------------
  Eigen::Vector3d SharedControlVelocityController::computeBaselineAngularVelocity(
      const Eigen::Vector3d &current_position, const Eigen::Vector3d &linear_velocity) const
  {
    const double x_E = current_position.x();
    const double y_E = current_position.y();
    const double v_x = linear_velocity.x();
    const double v_y = linear_velocity.y();
    const double denom = x_E * x_E + y_E * y_E;

    double omega_z_baseline = 0.0;
    if (denom > 0.0)
    {
      omega_z_baseline = (x_E * v_y - y_E * v_x) / denom;
    }

    return Eigen::Vector3d(0.0, 0.0, omega_z_baseline); // ω_{E,u,T}
  }

  // -----------------------------------------------------------------------------
  // sigmaD()
  // -----------------------------------------------------------------------------
  // Implements the σ_d(d) distance-based blending gate used in Eq. (5):
  //
  //   σ_d(d) = 0                  for d ≤ r_near
  //          = (d - r_near)/span  for r_near < d < r_far
  //          = 1                  for d ≥ r_far
  //
  // where r_near = min(r₁, r₂) and r_far = max(r₁, r₂).
  //
  // This gate allows a smooth transition between pure user control (σ_d=0) and
  // full shared control (σ_d=1) depending on the distance to the target.
  //
  // Degenerate case: if r₁ == r₂, σ_d becomes a binary step at r_near.
  // -----------------------------------------------------------------------------
  double SharedControlVelocityController::sigmaD(double d) const
  {
    const double r_near = std::min(r1_, r2_);
    const double r_far = std::max(r1_, r2_);
    const double span = r_far - r_near;

    // Degenerate: r1_ == r2_ : step gate at r_near
    if (span == 0.0)
    {
      static bool logged_once = false;
      if (!logged_once)
      {
        RCLCPP_INFO(get_node()->get_logger(),
                    "[sigmaD] Degenerate radii: r1 == r2 == %.3f m. "
                    "Using step gate at r=%.3f (σ_d=0 for d<=r, 1 for d>r).",
                    r1_, r_near);
        logged_once = true;
      }
      return (d > r_near) ? 1.0 : 0.0;
    }

    if (d <= r_near)
      return 0.0;
    if (d >= r_far)
      return 1.0;

    return (d - r_near) / span; // linear ramp for r_near < d < r_far
  }

  // -----------------------------------------------------------------------------
  // sigmaR()
  // -----------------------------------------------------------------------------
  // Implements the σ_r(ε_r) angular blending gate used in Eq. (18)
  // to control how much rotational assistance is applied.
  //
  // Definition:
  //     σ_r(ε_r) = 0                     for ε_r ≤ θ₂
  //               = (ε_r - θ₂)/(θ₁ - θ₂)  for θ₂ < ε_r < θ₁
  //               = 1                     for ε_r ≥ θ₁
  //
  // where:
  //   • ε_r  = current orientation error angle [rad]
  //   • θ₁, θ₂ = angular thresholds defining the transition region
  //               (typically θ₁ > θ₂)
  //
  // The gate increases smoothly from 0 to 1 as the orientation
  // error ε_r grows, allowing gradual blending between user-only
  // control (σ_r = 0) and shared control (σ_r = 1).
  //
  // Degenerate case:
  //   If θ₁ ≤ θ₂, the function logs a warning and returns σ_r = 0.
  // -----------------------------------------------------------------------------
  double SharedControlVelocityController::sigmaR(double epsilon_r) const
  {
    const double denominator = theta1_ - theta2_;
    if (denominator <= 0.0)
    {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[sigmaR] Invalid θ gate: theta1=%.3f ≤ theta2=%.3f. Forcing σ_r=0.", theta1_,
                  theta2_);
      return 0.0;
    }

    if (epsilon_r <= theta2_)
      return 0.0;
    if (epsilon_r >= theta1_)
      return 1.0;

    return (epsilon_r - theta2_) / denominator; // linear ramp
  }

  // -----------------------------------------------------------------------------
  // Shared-control shaping for MODE T (Translation)
  //
  // Equations:
  //   (5)  v_E = v_{E,u,T} + σ_d · (γ − 1) · (û ûᵀ) · v_{E,u,T}
  //   (15) ω_E = (1 − σ_d) · ω_{E,u,T} + σ_d · (‖v_E‖ · (ε_r / (ε_d − r₂)) · ŵ)
  //
  // Inputs:
  //   - soft_goal.x : goal position x_G
  //   - soft_goal.q : goal orientation q_G
  //   - current_position    : end-effector position x_E (in base frame)
  //   - current_orientation : end-effector orientation q_E (in base frame)
  //   - initial_filtered_linear_velocity : user linear command v_J (filtered in base frame)
  //
  // Output:
  //   - pair { linear_velocity_assisted v_E , omega_assisted ω_E }
  //
  // Notes:
  //   • σ_d(ε_d) gates assistance based on distance to the goal.
  //   • Only the goal-aligned velocity component v_∥ is scaled by γ; v_⊥ is preserved.
  //   • ω_E blends between the baseline wrist alignment ω_{E,u,T} and orientation-driven
  //     assistance proportional to ε_r / (ε_d − r₂).
  // -----------------------------------------------------------------------------
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applySharedControlModeT(const Goal &soft_goal, const Eigen::Vector3d &current_position,
                              const Eigen::Quaterniond &current_orientation,
                              const Eigen::Vector3d &initial_filtered_linear_velocity)
  {
    // --- Distance & direction to soft goal ---
    const auto [epsilon_d, unit_axis_u] =
        computeDistanceAndDirection(soft_goal.x, current_position);

    // --- σ_d(ε_d) gate ---
    const double sigma = sigmaD(epsilon_d);

    // ------------------------------------------------------------
    // Eq. (5) — Linear assistance: scales the goal-aligned component of v_E by gamma,
    // blending smoothly from user-only (σ_d=0) to full assistance (σ_d=1).
    // The orthogonal component of the velocity is preserved explicitly so that only v_prallel is
    // scaled by gamma (not 1 + gamma).
    // ------------------------------------------------------------
    const Eigen::Vector3d v_parallel =
        initial_filtered_linear_velocity.dot(unit_axis_u) * unit_axis_u;
    const Eigen::Vector3d linear_velocity_assisted =
        initial_filtered_linear_velocity + sigma * (gamma_ - 1.0) * v_parallel;

    // --- Baseline angular command ω_{E,u,T} (agnostic mode T) ---
    const Eigen::Vector3d omega_baseline =
        computeBaselineAngularVelocity(current_position, linear_velocity_assisted);

    // ------------------------------------------------------------
    // Orientation error: log(q_G q_E^{-1}) = ε_r * ŵ
    // (angle ε_r and unit axis ŵ)
    // ------------------------------------------------------------
    const RotationError rotation_error = computeRotationError(soft_goal.q, current_orientation);
    const double epsilon_r = rotation_error.angle;
    const Eigen::Vector3d unit_axis_w =
        (epsilon_r > 0.0) ? rotation_error.axis : Eigen::Vector3d::Zero();

    // ------------------------------------------------------------
    // Eq. (5) — Angular shaping:
    //   ω_E = (1 - σ_d) * ω_{E,u,T} + σ_d * ( ||v_E|| * (ε_r / ε_d - r2) * ŵ )
    // Edge cases:
    //   - If ed→0 ⇒ σ_d=0 ⇒ ω_E = ω_{E,u,T}
    // ------------------------------------------------------------
    Eigen::Vector3d omega_assisted = omega_baseline;

    if (sigma > 0.0)
    {
      if (epsilon_d <= 0.0)
      {
        // Degenerate case: goal reached in translation ⇒ σ=0 should already hold,
        // but if we get here numerically, keep baseline and inform.
        RCLCPP_INFO(get_node()->get_logger(),
                    "[applySharedControlModeT] Degenerate case: ed ≈ 0 ⇒ σ_d=0. Using baseline ω.");
      }
      else
      {
        const double gain = linear_velocity_assisted.norm() *
                            (epsilon_r / (epsilon_d - r2_)); // ||v_E|| * (ε_r / (ε_d - r2)) [rad/s]
        omega_assisted = (1.0 - sigma) * omega_baseline + sigma * gain * unit_axis_w;
        ;
      }
    }

    return {linear_velocity_assisted, omega_assisted};
  }

  // -----------------------------------------------------------------------------
  // Shared-control shaping for MODE W (Rotation-only)
  //
  // Equations:
  //   (18) σ_r(ε_r) = sat(0,1, (ε_r - θ2) / (θ1 - θ2))
  //   (19) ω_E       = ω_{E,u,W} + γ · σ_r · (ŵ ŵᵀ) · ω_{E,u,W}
  //   (20) v_E       = (x_G - x_E) × ω_E
  //
  // Inputs:
  //   - soft_goal.x : goal position x_G
  //   - soft_goal.q : goal orientation q_G
  //   - current_position    : end-effector position x_E (in base frame)
  //   - current_orientation : end-effector orientation q_E (in base frame)
  //   - initial_filtered_angular_velocity_base : user angular command ω_{E,u,W} (already in base
  //   frame)
  //
  // Output:
  //   - pair { linear_velocity_assisted v_E , omega_assisted ω_E }
  //
  // Notes:
  //   • ε_r and ŵ come from the shortest-path quaternion error log(q_G · q_E^{-1}).
  //   • σ_r gates assistance by orientation error (no help near alignment).
  //   • Linear velocity is the reorientation-only term so the EE rotates about x_G.
  // -----------------------------------------------------------------------------
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applySharedControlModeW(const Goal &soft_goal, const Eigen::Vector3d &current_position,
                              const Eigen::Quaterniond &current_orientation,
                              const Eigen::Vector3d &initial_filtered_angular_velocity_base)
  {
    // ------------------------------------------------------------
    // Eq. (16) (17) Orientation error: log(q_G q_E^{-1}) = ε_r * ŵ
    // (angle ε_r and unit axis ŵ)
    // ------------------------------------------------------------
    const RotationError rotation_error = computeRotationError(soft_goal.q, current_orientation);
    const double epsilon_r = rotation_error.angle;
    const Eigen::Vector3d unit_axis_w =
        (epsilon_r > 0.0) ? rotation_error.axis : Eigen::Vector3d::Zero();

    // --- Eq. (18) σ_r(ε_r) gate ---
    const double sigma_r = sigmaR(epsilon_r);

    // --- Eq. (19): amplify ω along ŵ ---
    Eigen::Vector3d omega_assisted = initial_filtered_angular_velocity_base; // ω_{E,u,W}
    if (epsilon_r > 0.0)
    {
      const Eigen::Vector3d omega_parallel =
          (initial_filtered_angular_velocity_base.dot(unit_axis_w)) * unit_axis_w;
      omega_assisted += gamma_ * sigma_r * omega_parallel;
    }

    // --- Eq. (20): reorientation-only translation ---
    const Eigen::Vector3d position_delta = (soft_goal.x - current_position);
    const Eigen::Vector3d linear_velocity_assisted = position_delta.cross(omega_assisted);

    return {linear_velocity_assisted, omega_assisted};
  }

  // -----------------------------------------------------------------------------
  // applyVelocitySaturation()
  // -----------------------------------------------------------------------------
  // Limits the rate of change of both linear and angular Cartesian velocities
  // between consecutive control cycles to prevent discontinuities or jerks.
  //
  // Implements element-wise saturation on each velocity component:
  //
  //   v_sat[i] = v_prev[i] + clamp(v_curr[i] - v_prev[i],
  //                                -Δv_max, +Δv_max)
  //
  // where:
  //   • v_prev : previous (already sent) velocity vector
  //   • v_curr : newly computed velocity vector
  //   • Δv_max : maximum allowed increment per control step
  //
  // Inputs:
  //   - current_linear / current_angular : current velocity commands
  //   - previous_linear / previous_angular : previous commanded velocities
  //   - max_linear_delta / max_angular_delta : per-axis limits [m/s] / [rad/s]
  //
  // Returns:
  //   pair { saturated_linear, saturated_angular }
  //
  // Notes:
  //   • Prevents acceleration spikes and discontinuities caused by numerical noise
  //     or abrupt joystick inputs.
  // -----------------------------------------------------------------------------
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SharedControlVelocityController::
      applyVelocitySaturation(const Eigen::Vector3d &current_linear,
                              const Eigen::Vector3d &previous_linear, const double max_linear_delta,
                              const Eigen::Vector3d &current_angular,
                              const Eigen::Vector3d &previous_angular,
                              const double max_angular_delta)
  {
    Eigen::Vector3d saturated_linear = previous_linear;
    Eigen::Vector3d saturated_angular = previous_angular;

    for (int i = 0; i < 3; ++i)
    {
      const double delta_linear =
          std::clamp(current_linear[i] - previous_linear[i], -max_linear_delta, max_linear_delta);
      const double delta_angular = std::clamp(current_angular[i] - previous_angular[i],
                                              -max_angular_delta, max_angular_delta);

      saturated_linear[i] += delta_linear;
      saturated_angular[i] += delta_angular;
    }

    return {saturated_linear, saturated_angular};
  }

  // -----------------------------------------------------------------------------
  // publishDebugData()
  // -----------------------------------------------------------------------------
  // Publishes all relevant debug signals for visualization and logging,
  // including raw, guided, and filtered velocities, goal confidences,
  // and poses (soft and agnostic goals) for RViz and PlotJuggler.
  // -----------------------------------------------------------------------------
  void SharedControlVelocityController::publishDebugData(
      const geometry_msgs::msg::Twist &latest_twist,
      const Eigen::Vector3d &initial_filtered_linear_velocity,
      const Eigen::Vector3d &initial_filtered_angular_velocity,
      const Eigen::Vector3d &cartesian_linear_velocity,
      const Eigen::Vector3d &cartesian_angular_velocity,
      const Eigen::Vector3d &filtered_linear_velocity,
      const Eigen::Vector3d &filtered_angular_velocity, const Goal &soft_goal)
  {
    // Early exit if nothing to publish
    if (!latest_twist_pub_ && !initial_filtered_linear_velocity_pub_ &&
        !initial_filtered_angular_velocity_pub_ && !assisted_linear_velocity_pub_ &&
        !assisted_angular_velocity_pub_ && !filtered_linear_velocity_pub_ &&
        !filtered_angular_velocity_pub_ && !soft_goal_pub_ && !agnostic_goal_pub_ && !conf_pub_)
    {
      return;
    }

    const auto now = get_node()->now();

    auto to_vec3 = [](const Eigen::Vector3d &v) {
      geometry_msgs::msg::Vector3 m;
      m.x = v.x();
      m.y = v.y();
      m.z = v.z();
      return m;
    };

    // Latest twist (already a Twist)
    if (latest_twist_pub_)
      latest_twist_pub_->publish(latest_twist);

    // Initial filtered velocities
    if (initial_filtered_linear_velocity_pub_)
      initial_filtered_linear_velocity_pub_->publish(to_vec3(initial_filtered_linear_velocity));
    if (initial_filtered_angular_velocity_pub_)
      initial_filtered_angular_velocity_pub_->publish(to_vec3(initial_filtered_angular_velocity));

    // Assisted linear / angular velocities
    if (assisted_linear_velocity_pub_)
      assisted_linear_velocity_pub_->publish(to_vec3(cartesian_linear_velocity));
    if (assisted_angular_velocity_pub_)
      assisted_angular_velocity_pub_->publish(to_vec3(cartesian_angular_velocity));

    // Post-saturation velocities
    if (filtered_linear_velocity_pub_)
      filtered_linear_velocity_pub_->publish(to_vec3(filtered_linear_velocity));
    if (filtered_angular_velocity_pub_)
      filtered_angular_velocity_pub_->publish(to_vec3(filtered_angular_velocity));

    // Soft goal pose
    if (soft_goal_pub_)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = now;
      ps.header.frame_id = "base";
      ps.pose.position.x = soft_goal.x.x();
      ps.pose.position.y = soft_goal.x.y();
      ps.pose.position.z = soft_goal.x.z();
      ps.pose.orientation.w = soft_goal.q.w();
      ps.pose.orientation.x = soft_goal.q.x();
      ps.pose.orientation.y = soft_goal.q.y();
      ps.pose.orientation.z = soft_goal.q.z();
      soft_goal_pub_->publish(ps);
    }

    // Goal confidences
    if (conf_pub_)
    {
      std_msgs::msg::Float64MultiArray cmsg;
      cmsg.data.reserve(goals_.size());
      for (const auto &g : goals_)
        cmsg.data.push_back(g.c);
      conf_pub_->publish(cmsg);
    }

    // Agnostic goal (G0)
    if (agnostic_goal_pub_ && !goals_.empty())
    {
      geometry_msgs::msg::PoseStamped g0;
      g0.header.stamp = now;
      g0.header.frame_id = "base";
      g0.pose.position.x = goals_[0].x.x();
      g0.pose.position.y = goals_[0].x.y();
      g0.pose.position.z = goals_[0].x.z();
      g0.pose.orientation.w = goals_[0].q.w();
      g0.pose.orientation.x = goals_[0].q.x();
      g0.pose.orientation.y = goals_[0].q.y();
      g0.pose.orientation.z = goals_[0].q.z();
      agnostic_goal_pub_->publish(g0);
    }
  }

  // -----------------------------------------------------------------------------
  // checkKnownGoalsAndBeep()
  // -----------------------------------------------------------------------------
  // Monitors whether the end-effector is within 6D reach of any known goal.
  // A goal is considered reached when both distance ≤ ε_t and orientation error ≤ ε_r
  // for at least dwell_ms milliseconds. When reached, a one-shot beep and goal index
  // are published; otherwise, a zero flag is sent each cycle. Prevents repeated triggers
  // using a dwell and latch mechanism.
  // -----------------------------------------------------------------------------
  void SharedControlVelocityController::checkKnownGoalsAndBeep(const rclcpp::Time &now)
  {
    // Default: publish 0 (no reach) each cycle unless we actually fire a beep
    if (goal_reached_pub_)
    {
      std_msgs::msg::Int32 flag;
      flag.data = 0;
      goal_reached_pub_->publish(flag);
    }

    // Quick exit if no known goals configured
    if (known_goals_poses_param_.empty())
    {
      // reset state
      last_reached_goal_idx_ = -1;
      beeped_for_this_visit_ = false;
      return;
    }

    /* -- Find the best candidate inside reach (min distance among those that pass thresholds) -- */
    int best_goal_idx = -1;
    double best_distance = std::numeric_limits<double>::infinity();

    // Strict 6D check: both translation and orientation must be within thresholds
    for (size_t k = 0; k < known_goals_poses_param_.size(); ++k)
    {
      const auto &goal = known_goals_poses_param_[k];

      const Eigen::Vector3d goal_position(goal[0], goal[1], goal[2]);
      Eigen::Quaterniond goal_orientation(goal[6], goal[3], goal[4], goal[5]); // (w,x,y,z)
      if (goal_orientation.norm() < 1e-9)
        goal_orientation = Eigen::Quaterniond::Identity();
      else
        goal_orientation.normalize();

      const double distance = (goal_position - current_position_).norm();
      const bool isTransOK = (distance <= eps_t_reach_);

      const double orientation_error =
          computeRotationError(goal_orientation, current_orientation_).angle;
      const bool isRotOK = (orientation_error <= eps_r_reach_rad_);

      if (isTransOK && isRotOK)
      {
        if (distance < best_distance)
        {
          best_distance = distance;
          best_goal_idx = static_cast<int>(k);
        }
      }
    }

    /* -- Dwell state machine -- */
    if (best_goal_idx >= 0)
    {
      // Starting a new candidate?
      if (last_reached_goal_idx_ != best_goal_idx)
      {
        last_reached_goal_idx_ = best_goal_idx;
        reached_enter_time_ = now;
        beeped_for_this_visit_ = false;
      }

      // Already dwelling on this candidate: check elapsed time
      if (!beeped_for_this_visit_)
      {
        const double elapsed_ms = (now - reached_enter_time_).nanoseconds() / 1'000'000.0;
        if (elapsed_ms >= static_cast<double>(dwell_ms_))
        {
          // Fire one-shot beep + pulse flag
          if (beep_pub_)
          {
            std_msgs::msg::Bool isGoalReached;
            isGoalReached.data = true;
            beep_pub_->publish(
                isGoalReached); // topic: /feedback/beep_trigger --> python node listens to this
                                // node and launches the sound feedback
          }
          if (goal_reached_pub_)
          {
            std_msgs::msg::Int32 goal_index;
            goal_index.data = best_goal_idx + 1; // publish the index of the goal that was reached
            goal_reached_pub_->publish(goal_index);
          }

          // Log in console with optional label
          const bool has_label = (best_goal_idx < static_cast<int>(known_goals_labels_.size()) &&
                                  !known_goals_labels_[best_goal_idx].empty());
          const char *label = has_label ? known_goals_labels_[best_goal_idx].c_str() : nullptr;

          if (label)
          {
            RCLCPP_INFO(get_node()->get_logger(), "[REACHED] KG%u '%s' — dist=%.3f m, dwell=%d ms",
                        best_goal_idx + 1, label, best_distance, dwell_ms_);
          }
          else
          {
            RCLCPP_INFO(get_node()->get_logger(), "[REACHED] KG%u — dist=%.3f m, dwell=%d ms",
                        best_goal_idx + 1, best_distance, dwell_ms_);
          }

          beeped_for_this_visit_ = true; // latch while we remain inside
        }
      }
    }
    else
    {
      // no candidate inside reach: reset dwell/latch
      last_reached_goal_idx_ = -1;
      beeped_for_this_visit_ = false;
    }
  }

} // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::SharedControlVelocityController,
                       controller_interface::ControllerInterface)
