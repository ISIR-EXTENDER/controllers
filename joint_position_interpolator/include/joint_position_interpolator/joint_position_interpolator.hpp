#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "robot_interfaces/joint_position_component.hpp"

#include "extender_msgs/msg/joint_position_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cmath>

namespace joint_controllers
{
  /// @brief struct to hold useful informations to interpolate
  struct InterpolateData
  {
    /// @brief start position of the joint
    double start_pos;
    /// @brief adjusted end position (unwrapped if joint is continuous)
    double end_pos;
    /// @brief velocity for the interpolation
    double velocity;
  };

  /// @brief Class that implement a joint position interpolator. It allows to compute trajectory
  /// from the current position, to a desired position, with a constant velocity.
  class JointPositionInterpolator : public controller_interface::ControllerInterface
  {
  public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    typedef robot_interfaces::JointCommand JointCommand;
    typedef extender_msgs::msg::JointPositionCommand JointPositionMessage;

    JointPositionInterpolator();
    virtual ~JointPositionInterpolator();

    // Configure command and state interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// Main update loop called periodically by the controller manager
    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    // Lifecycle callbacks
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

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
    void setupSubscribers();
    void setupPublishers();
    bool setupRobotInterface();

    void activatePublishers();
    void deactivatePublishers();

    /// @brief Callback of the subscriber. Will store the desired position in the unordered_map
    /// target_positions_
    /// @param msg message send on the topic /desired_joint_positions
    void desiredPositionCallback(const JointPositionMessage::SharedPtr msg);

    /// @brief Compute all joint trajectories based on current state and desired position
    void computeTrajectory();


    /// @brief Generic component to interface with robot hardware
    std::unique_ptr<robot_interfaces::GenericJointPosition> robot_interface_;

    /// @brief Subscriber to get the desired position
    rclcpp::Subscription<JointPositionMessage>::SharedPtr desired_position_sub_;

    /// @brief Interpolation velocity
    double max_interpolation_velocity_ = 0.01; // rad/s, default value
    double elapsed_time_ = 0.0;
    double trajectory_duration_ = 0.0;
    bool trajectory_running_ = false;

    /// @brief target positions matched with the joint names
    std::unordered_map<std::string, double> target_positions_map_;
    Eigen::VectorXd start_positions_;
    Eigen::VectorXd target_positions_;
    /// @brief joint names of the robot
    std::vector<std::string> joint_names_;
    Eigen::VectorXd lowerLimits;
    Eigen::VectorXd upperLimits;
    Eigen::VectorXd velLimits;

    
  };
} // namespace joint_controllers