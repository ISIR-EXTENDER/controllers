#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <optional>
#include <cmath>

/** How many segments to draw the cone rim with */
static constexpr int N_CONE_POINTS = 12;

class GoalMarkerPublisher : public rclcpp::Node
{
public:
  GoalMarkerPublisher()
  : Node("goal_marker_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---------------- Parameters ----------------
    this->declare_parameter<std::vector<double>>("goal_positions", {0.3, 0.0, 0.3});
    this->declare_parameter<std::string>("base_frame", "base");
    this->declare_parameter<std::string>("ee_frame",   "fr3_hand_tcp");
    this->declare_parameter<double>("r1", 0.08);
    this->declare_parameter<double>("r2", 0.04);
    this->declare_parameter<double>("theta_l_deg", 10.0);
    this->declare_parameter<std::string>("conf_topic", "/debug/goal_confidences");
    this->declare_parameter<std::string>("soft_goal_topic", "/debug/soft_goal");         // PoseStamped
    this->declare_parameter<std::string>("agnostic_goal_topic", "/debug/agnostic_goal"); // PoseStamped for G0
    this->declare_parameter<std::string>("raw_velocity_topic", "/debug/raw_velocity");
    this->declare_parameter<double>("ve_scale", 5.0);          // scale factor to see vE arrow
    this->declare_parameter<bool>("draw_vectors", true);
    this->declare_parameter<double>("rate_hz", 10.0);
    // soft goal orientation axes
    this->declare_parameter<bool>("draw_soft_goal_axes", true);
    this->declare_parameter<double>("axes_len", 0.10);
    this->declare_parameter<double>("axes_shaft_d", 0.004);
    this->declare_parameter<double>("axes_head_d", 0.012);

    base_frame_          = this->get_parameter("base_frame").as_string();
    ee_frame_            = this->get_parameter("ee_frame").as_string();
    r1_                  = this->get_parameter("r1").as_double();
    r2_                  = this->get_parameter("r2").as_double();
    theta_l_rad_         = this->get_parameter("theta_l_deg").as_double() * M_PI / 180.0;
    conf_topic_          = this->get_parameter("conf_topic").as_string();
    soft_goal_topic_     = this->get_parameter("soft_goal_topic").as_string();
    agnostic_goal_topic_ = this->get_parameter("agnostic_goal_topic").as_string();
    raw_velocity_topic_  = this->get_parameter("raw_velocity_topic").as_string();
    ve_scale_            = this->get_parameter("ve_scale").as_double();
    draw_vectors_        = this->get_parameter("draw_vectors").as_bool();
    // soft goal axes
    draw_soft_goal_axes_ = this->get_parameter("draw_soft_goal_axes").as_bool();
    axes_len_            = this->get_parameter("axes_len").as_double();
    axes_shaft_d_        = this->get_parameter("axes_shaft_d").as_double();
    axes_head_d_         = this->get_parameter("axes_head_d").as_double();

    const auto vec = this->get_parameter("goal_positions").as_double_array();
    goals_.reserve(vec.size() / 3);
    for (size_t i = 0; i + 2 < vec.size(); i += 3) {
      goals_.emplace_back(vec[i], vec[i + 1], vec[i + 2]);
    }

    // ---------------- Publishers / Subs ----------------
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("goal_markers", 1);

    conf_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      conf_topic_, 1,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        last_confidences_ = msg->data;
      });

    // Soft goal (magenta) as PoseStamped
    soft_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      soft_goal_topic_, 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr p) {
        last_soft_goal_ = Eigen::Vector3d(p->pose.position.x,
                                          p->pose.position.y,
                                          p->pose.position.z);
        last_soft_goal_quat_ = Eigen::Quaterniond(
            p->pose.orientation.w,
            p->pose.orientation.x,
            p->pose.orientation.y,
            p->pose.orientation.z
        );
      });


    // Real agnostic goal (G0) from the controller
    agnostic_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      agnostic_goal_topic_, 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr p) {
        last_g0_pos_ = Eigen::Vector3d(p->pose.position.x,
                                       p->pose.position.y,
                                       p->pose.position.z);
      });

    raw_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      raw_velocity_topic_, 1,
      [this](const geometry_msgs::msg::Vector3::SharedPtr v) {
        last_vE_ = Eigen::Vector3d(v->x, v->y, v->z);
        have_vE_ = true;
      });

    const double rate_hz = this->get_parameter("rate_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, rate_hz))),
      std::bind(&GoalMarkerPublisher::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "GoalMarkerPublisher running. base='%s', ee='%s', agnostic_topic='%s'",
                base_frame_.c_str(), ee_frame_.c_str(), agnostic_goal_topic_.c_str());
  }

private:
  struct Goal {
    double x, y, z;
    Goal(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
  };

  // --------- Params / State ----------
  std::vector<Goal> goals_;
  std::vector<double> last_confidences_;            // could be [c0, c1..cN] or [c1..cN]
  std::optional<Eigen::Vector3d> last_soft_goal_;   // magenta sphere position
  std::optional<Eigen::Vector3d> last_g0_pos_;      // agnostic (G0) position from controller
  Eigen::Vector3d last_vE_{Eigen::Vector3d::Zero()};
  bool have_vE_{false};

  std::string base_frame_, ee_frame_;
  double r1_{0.08};
  double r2_{0.04};
  double theta_l_rad_{10.0 * M_PI / 180.0};
  std::string conf_topic_;
  std::string soft_goal_topic_;
  std::string agnostic_goal_topic_;
  std::string raw_velocity_topic_;
  double ve_scale_{5.0};
  bool draw_vectors_{true};
  // soft goal axes
  std::optional<Eigen::Quaterniond> last_soft_goal_quat_;
  bool   draw_soft_goal_axes_{true};
  double axes_len_{0.10}, axes_shaft_d_{0.004}, axes_head_d_{0.012};


  // --------- ROS infra ----------
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr conf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr soft_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agnostic_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr raw_velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ------------------------------------------------------------
  void onTimer()
  {
    geometry_msgs::msg::TransformStamped T;
    bool have_tf = tf_buffer_.canTransform(base_frame_, ee_frame_, tf2::TimePointZero,
                                           tf2::durationFromSec(0.02));
    if (!have_tf) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed (%s -> %s). Check frame names.",
                           base_frame_.c_str(), ee_frame_.c_str());
      publishMarkers(std::nullopt);
      return;
    }

    try {
      T = tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF exception: %s", ex.what());
      publishMarkers(std::nullopt);
      return;
    }

    Eigen::Vector3d ee_pos(T.transform.translation.x,
                           T.transform.translation.y,
                           T.transform.translation.z);

    publishMarkers(ee_pos);
  }

  // ------------------------------------------------------------
  int maxConfidenceIdxIncludingC0() const
  {
    if (last_confidences_.empty()) return -1;
    double best = -1.0;
    int best_idx = -1;
    for (size_t i = 0; i < last_confidences_.size(); ++i) {
      if (last_confidences_[i] > best) {
        best = last_confidences_[i];
        best_idx = static_cast<int>(i);
      }
    }
    return best_idx;
  }

  // ------------------------------------------------------------
  void publishMarkers(std::optional<Eigen::Vector3d> ee_pos_opt)
  {
    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time now = this->now();

    // Who has the highest c? (including c0 if present)
    const int max_c_global_idx = maxConfidenceIdxIncludingC0();

    const bool has_c0 = (last_confidences_.size() == goals_.size() + 1);
    bool agnostic_is_max = false;
    int  max_goal_local_idx = -1; // 0..goals_.size()-1
    if (max_c_global_idx >= 0) {
      if (has_c0) {
        if (max_c_global_idx == 0) {
          agnostic_is_max = true;
        } else {
          max_goal_local_idx = max_c_global_idx - 1;
        }
      } else {
        max_goal_local_idx = max_c_global_idx;
      }
    }

    // --------- 0) Agnostic goal (G0) ----------
    if (last_g0_pos_.has_value()) {
      const auto &agn = *last_g0_pos_;
      const double agn_diam = agnostic_is_max ? 0.08 : 0.05;

      auto m_agn = basicSphere(now, base_frame_, 0, agn, agn_diam);
      m_agn.ns = "agnostic_goal";
      m_agn.color.r = 0.2; m_agn.color.g = 0.6; m_agn.color.b = 1.0; m_agn.color.a = 1.0;
      arr.markers.push_back(m_agn);

      // label
      arr.markers.push_back(basicText(now, base_frame_, 10000,
                                      agn + Eigen::Vector3d(0,0,0.07), "(agnostic)"));

      // raw velocity arrow anchored at G0 for visibility
      if (draw_vectors_ && have_vE_) {
        Eigen::Vector3d tip = agn + ve_scale_ * last_vE_;
        arr.markers.push_back(basicArrow(now, base_frame_, 50000, agn, tip,
                                         0.004, 0.01,
                                         1.0, 0.0, 0.0, 0.9));
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No agnostic_goal PoseStamped received on '%s' yet; G0 marker not drawn.",
                           agnostic_goal_topic_.c_str());
    }

    // --------- 1) Discrete goals (spheres + r1) ----------
    for (size_t i = 0; i < goals_.size(); ++i) {
      const Eigen::Vector3d p(goals_[i].x, goals_[i].y, goals_[i].z);
      const bool is_max = (static_cast<int>(i) == max_goal_local_idx);

      // main goal sphere
      auto m = basicSphere(now, base_frame_, static_cast<int>(i + 1), p,
                           is_max ? 0.08 : 0.05);
      m.ns = "goals";
      if (is_max) {
        m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 1.0;   // highlight
      } else {
        m.color.r = 1.0; m.color.g = 0.4; m.color.b = 0.0; m.color.a = 0.8;
      }
      arr.markers.push_back(m);

      // r1 translucent sphere
      auto r1m = basicSphere(now, base_frame_, static_cast<int>(1000 + i), p, 2.0 * r1_);
      r1m.ns = "r1_spheres";
      r1m.color.r = 0.8; r1m.color.g = 0.8; r1m.color.b = 0.0; r1m.color.a = 0.20;
      arr.markers.push_back(r1m);

      // r2 translucent sphere (assistance gate σ_d=0 inside r2)
      auto r2m = basicSphere(now, base_frame_, static_cast<int>(11000 + i), p, 2.0 * r2_);
      r2m.ns = "r2_spheres";
      r2m.color.r = 0.0; r2m.color.g = 0.4; r2m.color.b = 1.0; r2m.color.a = 0.30; 
      arr.markers.push_back(r2m);

      // label "G<i>"
      auto txt = basicText(now, base_frame_, static_cast<int>(2000 + i),
                           p + Eigen::Vector3d(0,0,0.07),
                           "G" + std::to_string(i + 1));
      arr.markers.push_back(txt);

      // --------- 2) Cones (θ_l) from EE to each goal ----------
      if (ee_pos_opt.has_value()) {
        const auto &ee = *ee_pos_opt;
        double dist = (p - ee).norm();
        auto cone = makeCone(now, base_frame_, static_cast<int>(3000 + i),
                             ee, p, theta_l_rad_, dist);

        // Opaque if this goal is the max (and it's not agnostic); otherwise fade
        if (is_max && !agnostic_is_max) {
          cone.color.a = 0.5;
        } else {
          cone.color.a = 0.08;
        }
        arr.markers.push_back(cone);
      }
    }

    // --------- 3) u1 arrow (from EE to the most confident *real* goal) ----------
    if (draw_vectors_ && ee_pos_opt.has_value() && max_goal_local_idx >= 0) {
      const auto &ee = *ee_pos_opt;
      Eigen::Vector3d g(goals_[max_goal_local_idx].x, goals_[max_goal_local_idx].y, goals_[max_goal_local_idx].z);
      Eigen::Vector3d u = (g - ee);
      double n = u.norm();
      if (n > 1e-9) u /= n;

      // draw a cyan arrow of length equal to (distance to goal) for clarity
      Eigen::Vector3d tip = ee + u * (g - ee).norm();
      arr.markers.push_back(basicArrow(now, base_frame_, 60000,
                                       ee, tip, 0.004, 0.01,
                                       0.0, 1.0, 1.0, 0.9));
    }

    // --------- 4) Soft goal (magenta) ----------
    if (last_soft_goal_.has_value()) {
      const auto &sg = *last_soft_goal_;
      auto ms = basicSphere(now, base_frame_, 70000, sg, 0.06);
      ms.ns = "soft_goal";
      ms.color.r = 1.0; ms.color.g = 0.0; ms.color.b = 1.0; ms.color.a = 0.9;
      arr.markers.push_back(ms);
      arr.markers.push_back(basicText(now, base_frame_, 70001,
                                      sg + Eigen::Vector3d(0,0,0.07), "Soft"));
    }

    // Soft goal axes (if we have both position and orientation)
    if (draw_soft_goal_axes_ && last_soft_goal_.has_value() && last_soft_goal_quat_.has_value()) {
      addAxes(now, base_frame_, 80000,
              *last_soft_goal_, *last_soft_goal_quat_,
              axes_len_, axes_shaft_d_, axes_head_d_, arr);
    }


    marker_pub_->publish(arr);
  }

  // ------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------
  void addAxes(const rclcpp::Time &stamp,
    const std::string &frame,
    int id_base,
    const Eigen::Vector3d &p,
    const Eigen::Quaterniond &q,
    double len,
    double shaft_d,
    double head_d,
    visualization_msgs::msg::MarkerArray &arr) const
  {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Vector3d ex = R.col(0), ey = R.col(1), ez = R.col(2);

    auto add = [&](int id, const Eigen::Vector3d& dir, double r,double g,double b) {
    auto m = basicArrow(stamp, frame, id, p, p + len*dir, shaft_d, head_d, r,g,b, 1.0);
    m.ns = "soft_goal_axes";
    arr.markers.push_back(m);
    };

    add(id_base + 0, ex, 1.0, 0.0, 0.0); // X (red)
    add(id_base + 1, ey, 0.0, 1.0, 0.0); // Y (green)
    add(id_base + 2, ez, 0.0, 0.0, 1.0); // Z (blue)
  }

  
  visualization_msgs::msg::Marker basicSphere(const rclcpp::Time &stamp,
                                              const std::string &frame,
                                              int id,
                                              const Eigen::Vector3d &p,
                                              double diameter) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = diameter;
    m.color.a = 1.0;
    return m;
  }

  visualization_msgs::msg::Marker basicText(const rclcpp::Time &stamp,
                                            const std::string &frame,
                                            int id,
                                            const Eigen::Vector3d &p,
                                            const std::string &text) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.06; // text height
    m.color.r = m.color.g = m.color.b = 1.0;
    m.color.a = 0.9;
    m.text = text;
    return m;
  }

  visualization_msgs::msg::Marker basicArrow(const rclcpp::Time &stamp,
                                             const std::string &frame,
                                             int id,
                                             const Eigen::Vector3d &from,
                                             const Eigen::Vector3d &to,
                                             double shaft_diam,
                                             double head_diam,
                                             double r,double g,double b,double a) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.id = id;
    m.ns = "vectors";
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0.x = from.x(); p0.y = from.y(); p0.z = from.z();
    p1.x = to.x();   p1.y = to.y();   p1.z = to.z();
    m.points.push_back(p0);
    m.points.push_back(p1);

    // scale.x = shaft diameter, scale.y = head diameter, scale.z = head length (rviz semantics)
    m.scale.x = shaft_diam;
    m.scale.y = head_diam;
    m.scale.z = head_diam * 2.0;

    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  visualization_msgs::msg::Marker makeCone(const rclcpp::Time &stamp,
                                           const std::string &frame,
                                           int id,
                                           const Eigen::Vector3d &apex,
                                           const Eigen::Vector3d &goal,
                                           double theta,
                                           double length) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "cones";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.002;   // line width
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5;

    Eigen::Vector3d dir = goal - apex;
    double n = dir.norm();
    if (n < 1e-6) return m;
    dir /= n;

    // circle radius at the tip
    const double r = length * std::tan(theta);

    // orthonormal basis
    Eigen::Vector3d e1;
    if (std::abs(dir.z()) < 0.9)
      e1 = dir.cross(Eigen::Vector3d::UnitZ()).normalized();
    else
      e1 = dir.cross(Eigen::Vector3d::UnitY()).normalized();
    Eigen::Vector3d e2 = dir.cross(e1);

    std::vector<geometry_msgs::msg::Point> circle_pts(N_CONE_POINTS);
    for (int k = 0; k < N_CONE_POINTS; ++k) {
      double a = 2.0 * M_PI * k / N_CONE_POINTS;
      Eigen::Vector3d c = apex + dir * length + r * (std::cos(a) * e1 + std::sin(a) * e2);

      geometry_msgs::msg::Point pt;
      pt.x = c.x(); pt.y = c.y(); pt.z = c.z();
      circle_pts[k] = pt;

      // side lines (apex -> circle)
      geometry_msgs::msg::Point apex_pt;
      apex_pt.x = apex.x(); apex_pt.y = apex.y(); apex_pt.z = apex.z();
      m.points.push_back(apex_pt);
      m.points.push_back(pt);
    }

    // circle rim
    for (int k = 0; k < N_CONE_POINTS; ++k) {
      m.points.push_back(circle_pts[k]);
      m.points.push_back(circle_pts[(k + 1) % N_CONE_POINTS]);
    }

    return m;
  }
};

// ------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalMarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
