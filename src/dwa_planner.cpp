#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RobotState
{
  Pose2D pose;
  double v{0.0};
  double w{0.0};
  bool odom_ok{false};
  bool scan_ok{false};
  std::vector<float> ranges;
  float angle_min{0.0f};
  float angle_increment{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};
};

class SimpleDwa
{
public:
  struct Cmd
  {
    double v{0.0};
    double w{0.0};
  };

  struct Result
  {
    Cmd cmd;
    std::vector<std::vector<Pose2D>> sampled_trajs;
    std::vector<Pose2D> best_traj;
    bool valid{false};
  };

  void set_goal(const double x, const double y)
  {
    goal_x_ = x;
    goal_y_ = y;
  }

  void set_limits(const double max_v, const double max_w, const double max_acc_v, const double max_acc_w)
  {
    max_v_ = std::max(0.05, max_v);
    max_w_ = std::max(0.3, std::abs(max_w));
    max_acc_v_ = std::max(0.05, max_acc_v);
    max_acc_w_ = std::max(0.05, max_acc_w);
  }

  void set_sampling(const int v_samples, const int w_samples, const double dt, const double horizon)
  {
    v_samples_ = std::max(3, v_samples);
    w_samples_ = std::max(5, w_samples);
    dt_ = std::max(0.03, dt);
    horizon_ = std::max(0.5, horizon);
  }

  void set_weights(const double heading, const double clearance, const double speed)
  {
    heading_w_ = std::max(0.0, heading);
    clear_w_ = std::max(0.0, clearance);
    speed_w_ = std::max(0.0, speed);
  }

  Result plan(const RobotState & s) const
  {
    Result out;
    if (!s.odom_ok || !s.scan_ok || s.ranges.empty() || s.angle_increment == 0.0f) {
      return out;
    }

    const double min_v = 0.0;
    const double max_v = std::min(max_v_, s.v + max_acc_v_ * dt_);
    const double min_w = std::max(-max_w_, s.w - max_acc_w_ * dt_);
    const double max_w = std::min(max_w_, s.w + max_acc_w_ * dt_);

    double best_score = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < v_samples_; ++i) {
      const double a = (v_samples_ == 1) ? 0.0 : static_cast<double>(i) / (v_samples_ - 1);
      const double v = min_v + a * (max_v - min_v);

      for (int j = 0; j < w_samples_; ++j) {
        const double b = (w_samples_ == 1) ? 0.0 : static_cast<double>(j) / (w_samples_ - 1);
        const double w = min_w + b * (max_w - min_w);

        std::vector<Pose2D> traj = rollout(s.pose, v, w);
        if (traj.empty()) {
          continue;
        }
        out.sampled_trajs.push_back(traj);

        const double c = traj_min_clearance(s, traj);
        const double collision_limit = robot_radius_ + collision_buffer_;
        if (!std::isfinite(c) || c <= collision_limit) {
          continue;
        }

        const Pose2D & end = traj.back();
        const double heading = heading_score(end);
        const double clearance = std::clamp((c - collision_limit) / obstacle_alert_dist_, 0.0, 1.0);
        const double speed = std::clamp(v / max_v_, 0.0, 1.0);

        // Extra nudge: near obstacles, prefer sharper turns.
        const double obstacle_prox = 1.0 - clearance;
        const double turn_bonus = obstacle_prox * std::clamp(std::abs(w) / max_w_, 0.0, 1.0);

        const double score =
          heading_w_ * heading +
          clear_w_ * clearance +
          speed_w_ * speed +
          turn_w_ * turn_bonus;

        if (score > best_score) {
          best_score = score;
          out.cmd.v = v;
          out.cmd.w = w;
          out.best_traj = traj;
          out.valid = true;
        }
      }
    }

    return out;
  }

private:
  static double norm_angle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  std::vector<Pose2D> rollout(const Pose2D & start, const double v, const double w) const
  {
    std::vector<Pose2D> t;
    Pose2D p = start;
    for (double s = 0.0; s <= horizon_ + 1e-9; s += dt_) {
      p.x += v * std::cos(p.yaw) * dt_;
      p.y += v * std::sin(p.yaw) * dt_;
      p.yaw = norm_angle(p.yaw + w * dt_);
      t.push_back(p);
    }
    return t;
  }

  double traj_min_clearance(const RobotState & s, const std::vector<Pose2D> & traj) const
  {
    double best = std::numeric_limits<double>::infinity();
    bool seen = false;

    for (const auto & p : traj) {
      const double dx = p.x - s.pose.x;
      const double dy = p.y - s.pose.y;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-4) {
        continue;
      }

      const double rel = norm_angle(std::atan2(dy, dx) - s.pose.yaw);
      const double raw = (rel - static_cast<double>(s.angle_min)) /
        static_cast<double>(s.angle_increment);
      const int idx = static_cast<int>(std::lround(raw));
      if (idx < 0 || idx >= static_cast<int>(s.ranges.size())) {
        continue;
      }

      const float rr = s.ranges[static_cast<size_t>(idx)];
      double beam = std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(rr) && rr > s.range_min) {
        beam = static_cast<double>(rr);
      } else if (std::isinf(rr) && s.range_max > 0.0f) {
        beam = static_cast<double>(s.range_max);
      }

      if (std::isfinite(beam)) {
        seen = true;
        best = std::min(best, beam - dist);
      }
    }

    if (!seen) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return best;
  }

  double heading_score(const Pose2D & end) const
  {
    const double target = std::atan2(goal_y_ - end.y, goal_x_ - end.x);
    const double err = std::abs(norm_angle(target - end.yaw));
    return (M_PI - err) / M_PI;
  }

  double goal_x_{0.0};
  double goal_y_{0.0};

  double max_v_{0.22};
  double max_w_{2.5};
  double max_acc_v_{0.5};
  double max_acc_w_{3.5};

  int v_samples_{15};
  int w_samples_{41};
  double dt_{0.1};
  double horizon_{2.0};

  double robot_radius_{0.15};
  double collision_buffer_{0.10};
  double obstacle_alert_dist_{1.0};

  double heading_w_{0.35};
  double clear_w_{1.0};
  double speed_w_{0.20};
  double turn_w_{0.60};
};

class DwaPlannerNode : public rclcpp::Node
{
public:
  DwaPlannerNode() : Node("dwa_planner")
  {
    load_params();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DwaPlannerNode::on_odom, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DwaPlannerNode::on_scan, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&DwaPlannerNode::on_goal, this, std::placeholders::_1));
    goal_sub_legacy_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      legacy_goal_topic_, 10, std::bind(&DwaPlannerNode::on_goal, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_markers", 10);

    RCLCPP_INFO(get_logger(), "Simple DWA ready. Goal topics: %s , %s",
      goal_topic_.c_str(), legacy_goal_topic_.c_str());
  }

private:
  static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  void load_params()
  {
    const double max_v = declare_parameter<double>("max_vel_x", 0.20);
    const double max_w = declare_parameter<double>("max_rot_vel", 2.8);
    const double acc_v = declare_parameter<double>("acc_lim_x", 0.5);
    const double acc_w = declare_parameter<double>("acc_lim_theta", 4.0);
    const int v_samples = declare_parameter<int>("vx_samples", 15);
    const int w_samples = declare_parameter<int>("vth_samples", 41);
    const double sim_t = declare_parameter<double>("sim_time", 2.0);
    const double dt = declare_parameter<double>("sim_granularity", 0.1);

    const double heading_w = declare_parameter<double>("goal_distance_bias", 0.35);
    const double clear_w = declare_parameter<double>("occdist_scale", 1.0);
    const double speed_w = declare_parameter<double>("speed_cost_gain", 0.2);

    safety_stop_dist_ = declare_parameter<double>("safety_stop_distance", 0.30);
    safety_slow_dist_ = declare_parameter<double>("safety_slow_distance", 0.90);
    safety_turn_speed_ = declare_parameter<double>("safety_turn_speed", 1.8);

    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    legacy_goal_topic_ = declare_parameter<std::string>("legacy_goal_topic", "/move_base_simple/goal");

    planner_.set_limits(max_v, max_w, acc_v, acc_w);
    planner_.set_sampling(v_samples, w_samples, dt, sim_t);
    planner_.set_weights(heading_w, clear_w, speed_w);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    state_.pose.x = msg->pose.pose.position.x;
    state_.pose.y = msg->pose.pose.position.y;
    state_.pose.yaw = yaw_from_quat(msg->pose.pose.orientation);
    state_.v = msg->twist.twist.linear.x;
    state_.w = msg->twist.twist.angular.z;
    state_.odom_ok = true;
    run_once();
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    state_.ranges = msg->ranges;
    state_.angle_min = msg->angle_min;
    state_.angle_increment = msg->angle_increment;
    state_.range_min = msg->range_min;
    state_.range_max = msg->range_max;
    state_.scan_ok = true;
    run_once();
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    planner_.set_goal(msg->pose.position.x, msg->pose.position.y);
    goal_ok_ = true;
    RCLCPP_INFO(get_logger(), "Goal set: x=%.2f y=%.2f", msg->pose.position.x, msg->pose.position.y);
  }

  double front_min() const
  {
    if (state_.ranges.empty() || state_.angle_increment == 0.0f) {
      return 0.0;
    }

    const double half = 0.30;
    double best = std::numeric_limits<double>::infinity();
    bool found = false;

    for (size_t i = 0; i < state_.ranges.size(); ++i) {
      const double a = static_cast<double>(state_.angle_min) +
        static_cast<double>(i) * static_cast<double>(state_.angle_increment);
      if (a < -half || a > half) {
        continue;
      }

      const float r = state_.ranges[i];
      if (std::isfinite(r) && r > state_.range_min) {
        best = std::min(best, static_cast<double>(r));
        found = true;
      } else if (std::isinf(r) && state_.range_max > 0.0f) {
        best = std::min(best, static_cast<double>(state_.range_max));
        found = true;
      }
    }

    if (!found) {
      return 0.0;
    }
    return best;
  }

  double side_avg(const bool left) const
  {
    if (state_.ranges.empty() || state_.angle_increment == 0.0f) {
      return 0.0;
    }

    const double c = left ? 1.0 : -1.0;
    const double hw = 0.5;
    double sum = 0.0;
    int n = 0;

    for (size_t i = 0; i < state_.ranges.size(); ++i) {
      const double a = static_cast<double>(state_.angle_min) +
        static_cast<double>(i) * static_cast<double>(state_.angle_increment);
      if (a < (c - hw) || a > (c + hw)) {
        continue;
      }

      const float r = state_.ranges[i];
      if (std::isfinite(r) && r > state_.range_min) {
        sum += static_cast<double>(r);
        n++;
      } else if (std::isinf(r) && state_.range_max > 0.0f) {
        sum += static_cast<double>(state_.range_max);
        n++;
      }
    }

    if (n == 0) {
      return 0.0;
    }
    return sum / static_cast<double>(n);
  }

  geometry_msgs::msg::Twist safety_filter(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out = in;
    const double front = front_min();
    const double dir = (side_avg(true) >= side_avg(false)) ? 1.0 : -1.0;

    // Basic rule: if obstacle is near, stop and turn first.
    if (front > 0.0 && front < safety_slow_dist_) {
      out.linear.x = 0.0;
      out.angular.z = dir * safety_turn_speed_;
      return out;
    }

    if (front > 0.0 && front < safety_stop_dist_) {
      out.linear.x = 0.0;
      out.angular.z = dir * safety_turn_speed_;
    }

    return out;
  }

  void publish_markers(
    const std::vector<std::vector<Pose2D>> & sampled_trajs,
    const std::vector<Pose2D> & best_traj) const
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker all;
    all.header.frame_id = "odom";
    all.header.stamp = now();
    all.ns = "dwa_all";
    all.id = 0;
    all.type = visualization_msgs::msg::Marker::LINE_LIST;
    all.action = visualization_msgs::msg::Marker::ADD;
    all.scale.x = 0.015;
    all.color.r = 0.0f;
    all.color.g = 0.8f;
    all.color.b = 0.2f;
    all.color.a = 0.5f;
    for (const auto & traj : sampled_trajs) {
      if (traj.size() < 2) {
        continue;
      }
      for (size_t i = 1; i < traj.size(); ++i) {
        geometry_msgs::msg::Point p1;
        p1.x = traj[i - 1].x;
        p1.y = traj[i - 1].y;
        p1.z = 0.0;
        geometry_msgs::msg::Point p2;
        p2.x = traj[i].x;
        p2.y = traj[i].y;
        p2.z = 0.0;
        all.points.push_back(p1);
        all.points.push_back(p2);
      }
    }
    arr.markers.push_back(all);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = now();
    m.ns = "dwa_best";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.r = 1.0f;
    m.color.g = 0.1f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    for (const auto & p : best_traj) {
      geometry_msgs::msg::Point q;
      q.x = p.x;
      q.y = p.y;
      q.z = 0.0;
      m.points.push_back(q);
    }
    arr.markers.push_back(m);

    marker_pub_->publish(arr);
  }

  void run_once()
  {
    if (!state_.odom_ok || !state_.scan_ok) {
      return;
    }

    geometry_msgs::msg::Twist cmd;

    if (!goal_ok_) {
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500,
        "Waiting for 2D goal pose...");
      return;
    }

    const auto res = planner_.plan(state_);
    if (!res.valid) {
      cmd.linear.x = 0.0;
      cmd.angular.z = (side_avg(true) >= side_avg(false)) ? 1.2 : -1.2;
      cmd_pub_->publish(cmd);
      return;
    }

    publish_markers(res.sampled_trajs, res.best_traj);

    cmd.linear.x = res.cmd.v;
    cmd.angular.z = res.cmd.w;
    cmd = safety_filter(cmd);

    // small smoothing
    if (!has_last_) {
      last_ = cmd;
      has_last_ = true;
    } else {
      cmd.linear.x = 0.5 * cmd.linear.x + 0.5 * last_.linear.x;
      cmd.angular.z = 0.5 * cmd.angular.z + 0.5 * last_.angular.z;
      last_ = cmd;
    }

    cmd.linear.x = clamp(cmd.linear.x, -0.05, 0.22);
    cmd.angular.z = clamp(cmd.angular.z, -3.2, 3.2);

    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Assigned speed v=%.2f w=%.2f | Current position x=%.2f y=%.2f",
      cmd.linear.x,
      cmd.angular.z,
      state_.pose.x,
      state_.pose.y);
  }

  RobotState state_;
  SimpleDwa planner_;

  bool goal_ok_{false};
  bool has_last_{false};
  geometry_msgs::msg::Twist last_;

  double safety_stop_dist_{0.30};
  double safety_slow_dist_{0.90};
  double safety_turn_speed_{1.8};

  std::string goal_topic_{"/goal_pose"};
  std::string legacy_goal_topic_{"/move_base_simple/goal"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_legacy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
