#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include "joystick_interface/msg/teleop_cmd.hpp"
#include "robot_interfaces/generic_component.hpp"
#include "robot_interfaces/robot_interfaces_algos.hpp"

namespace cartesian_velocity_controller
{
  struct Goal
  {
    Eigen::Vector3d x;    // desired position
    Eigen::Quaterniond q; // desired orientation
    double c;             // confidence ∈ [0,1]
  };

  struct RotationError
  {
    double angle;                    // ε_r ∈ [0, π]
    Eigen::Vector3d axis;            // ŵ (zero if angle == 0)
    Eigen::Vector3d rotation_vector; // e_R = angle * axis (log map), zero if angle == 0
  };

  // Cartesian velocity controller using kinematic guides
  class SharedControlVelocityController : public controller_interface::ControllerInterface
  {
  public:
    SharedControlVelocityController();
    virtual ~SharedControlVelocityController();

    // Configure command and state interfaces.
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // Main update loop called periodically by the controller manager.
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
    uint8_t mode_{joystick_interface::msg::TeleopCmd::TRANSLATION_ROTATION}; // default to Translation_Rotation

    // Callback to receive Twist + Mode commands from the teleop node using a
    // custom msg.
    void teleopCmdCallback(const joystick_interface::msg::TeleopCmd::SharedPtr msg);

    RotationError computeRotationError(const Eigen::Quaterniond &q_goal,
                                       const Eigen::Quaterniond &q_current) const;

    /// Applies a first-order low-pass filter to a 3D vector.
    /// y[n] = α x[n] + (1 - α) y[n-1]
    Eigen::Vector3d applyLowPassFilterVector(const Eigen::Vector3d &input,
                                             const Eigen::Vector3d &previous, double alpha);

    /// Compute distance ε_d and unit direction û from EE to goal.
    /// Returns {ε_d, û}, with û = UnitX if ε_d ≈ 0.
    std::pair<double, Eigen::Vector3d> computeDistanceAndDirection(
        const Eigen::Vector3d &goal_position, const Eigen::Vector3d &current_position) const;

    /// Compute baseline angular velocity ω_{E,u,T} to keep the wrist
    /// aligned with the mobile frame in translation mode (MODE T).
    Eigen::Vector3d computeBaselineAngularVelocity(const Eigen::Vector3d &current_position,
                                                   const Eigen::Vector3d &linear_velocity) const;

    /// Compute σ_d(d) gate that blends between no-assistance and full-assistance.
    /// Returns 0 for d ≤ r_near, 1 for d ≥ r_far, and linearly interpolates
    /// otherwise.
    double sigmaD(double d) const;

    /// Compute the σ_r(ε_r) angular gate to modulate rotational assistance
    /// between the near (θ₂) and far (θ₁) thresholds.
    /// Returns 0 for ε_r ≤ θ₂, 1 for ε_r ≥ θ₁, and linearly interpolates
    /// otherwise.
    double sigmaR(double epsilon_r) const;

    // MODE T shared-control shaping (Eq. 5)
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applySharedControlModeT(
        const Goal &soft_goal, const Eigen::Vector3d &current_position,
        const Eigen::Quaterniond &current_orientation,
        const Eigen::Vector3d &initial_filtered_linear_velocity);

    /// Shared-control shaping for rotation mode (MODE W)
    /// to blend user angular commands with orientation-based assistance.
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applySharedControlModeW(
        const Goal &soft_goal, const Eigen::Vector3d &current_position,
        const Eigen::Quaterniond &current_orientation,
        const Eigen::Vector3d &initial_filtered_angular_velocity_base);

    // Velocity saturation
    std::pair<Eigen::Vector3d, Eigen::Vector3d> applyVelocitySaturation(
        const Eigen::Vector3d &current_linear, const Eigen::Vector3d &previous_linear,
        const double max_linear_delta, const Eigen::Vector3d &current_angular,
        const Eigen::Vector3d &previous_angular, const double max_angular_delta);

    // Publish debug data to plot
    void publishDebugData(const geometry_msgs::msg::Twist &latest_twist,
                          const Eigen::Vector3d &initial_filtered_linear_velocity,
                          const Eigen::Vector3d &initial_filtered_angular_velocity,
                          const Eigen::Vector3d &cartesian_linear_velocity,
                          const Eigen::Vector3d &cartesian_angular_velocity,
                          const Eigen::Vector3d &filtered_linear_velocity,
                          const Eigen::Vector3d &filtered_angular_velocity, const Goal &soft_goal);

    // Franka Cartesian velocity interface used for sending Cartesian velocity
    // commands.
    std::unique_ptr<robot_interfaces::GenericComponent> robot_vel_interface_;

    // Stores the latest Twist command received.
    geometry_msgs::msg::Twist latest_twist_;

    // Stores the previous initial filtered velocities
    Eigen::Vector3d previous_initial_filtered_linear_velocity_;
    Eigen::Vector3d previous_initial_filtered_angular_velocity_;

    // Used for final velocity saturation
    Eigen::Vector3d previous_cartesian_linear_velocity_;
    Eigen::Vector3d previous_cartesian_angular_velocity_;

    // Stores the previous post-filter velocities
    Eigen::Vector3d previous_filtered_linear_velocity_;
    Eigen::Vector3d previous_filtered_angular_velocity_;

    // Subscription for teleop Twist + Mode commands.
    rclcpp::Subscription<joystick_interface::msg::TeleopCmd>::SharedPtr teleop_cmd_sub_;

    // Robot state
    Eigen::Quaterniond current_orientation_;
    Eigen::Vector3d current_position_;

    // -- Multiple targets
    // Defined goals by the user: [x y z qx qy qz qw] × N
    std::vector<std::array<double, 7>> goals_poses_param_;

    // Known goals by the robot: [x y z qx qy qz qw] × N
    std::vector<std::array<double, 7>> known_goals_poses_param_;
    std::vector<std::string> known_goals_labels_;

    // --

    /* ---------- goal handling ---------- */
    // Data
    std::vector<Goal> goals_;

    // Functions for goal handling
    void initialiseGoals();
    void updateConfidences(double dt, const Eigen::Vector3d &vE);
    Goal computeSoftGoal() const;
    // Feedback function (ICRA Experiment): check known goals, apply dwell, emit
    // beep (and 1-cycle pulse on goal_reached_pub_)
    void checkKnownGoalsAndBeep(const rclcpp::Time &now);

    // Parameters
    double alpha_conf_;    // α  – confidence integration gain  [s-1]
    double theta_l_;       // θ_l – half-cone aperture          [rad]
    double r1_;            // r1 – inner dead-zone radius       [m]
    double v_j_max_{0.04}; // max velocity fine manipulation    [m/s]
    double gamma_{4.0};    // assistance gain on the parallel component
    double r2_{0.20};      // outer radius for σ_d(d) where assistance saturates to 1
    double theta1_;        // used in gate for assistance mode W
    double theta2_;        // used in gate for assistance mode W

    // Reach thresholds + dwell
    double eps_t_reach_{0.01};                   // meters (default 1 cm)
    double eps_r_reach_rad_{5.0 * M_PI / 180.0}; // radians (default 5 deg)
    int dwell_ms_{300};                          // milliseconds

    // Dwell tracking
    int last_reached_goal_idx_{-1};     // index in known_goals_
    rclcpp::Time reached_enter_time_;   // when we first met the condition
    bool beeped_for_this_visit_{false}; // edge-trigger guard

    /* ---------------------------------- */

    // Flags to activate or desactivate functions
    bool enable_shared_control_assistance_;
    bool enable_velocity_saturation_;
    bool enable_debug_publish_;

    // Low-pass filters
    double initial_filter_cutoff_frequency_;
    double final_filter_cutoff_frequency_;
    bool lpf_initialized_;

    // Cartesian velocity saturation
    double max_linear_delta_;
    double max_angular_delta_;

    // Input twist frame selection: "base" or "ee"
    std::string input_twist_frame_{"base"};

    // Plotting publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr latest_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr initial_filtered_linear_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
        initial_filtered_angular_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr assisted_linear_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr assisted_angular_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_linear_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_angular_velocity_pub_;

    // Goals and confidences publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr conf_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr soft_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr agnostic_goal_pub_;
    // Publishers for python node
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr beep_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_reached_pub_; // 0/1 flag
    // Mode publisher
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  };

} // namespace cartesian_velocity_controller
