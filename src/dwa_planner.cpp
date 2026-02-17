#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits> 
#include <chrono> 

struct RobotState{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double lin_vel{0.0};
    double ang_vel{0.0};

};

struct DWAConfig{
    double max_vel;
    double max_w;
    double max_lin_acc;
    double max_ang_acc;
    double dt;
    double predict_time;
    int v_samples;
    int w_samples;
};

struct DynamicWindow{
    double v_min;
    double v_max;
    double w_min;
    double w_max;
};

struct control{
    double v;
    double w;
};

struct Obstacle{
    double x;
    double y;
};

class DWAPlanner{
private:
    DWAConfig config_;
    std::vector <Obstacle> obstacles_;
    double robot_radius_{0.25};

public:
    struct PlanResult{
        control best_control;
        std::vector<std::vector<RobotState>> sampled_trajs;
        std::vector<RobotState> best_traj;
        bool valid{false};
    };

    DWAPlanner(const DWAConfig& cfg) : config_(cfg){}

    void updateObstacles(const std::vector<Obstacle>& obs){
        obstacles_ = obs;
    }

    void setRobotRadius(double radius){
        robot_radius_ = radius;
    }

    double Normalisation(double x) const{
        return std::max(0.0, std::min(1.0, x));
    }

    DynamicWindow ComputeDynamicWindow(const RobotState& state){
        DynamicWindow window;
        window.v_min = std::max(0.0, state.lin_vel - config_.max_lin_acc * config_.dt);
        window.v_max = std::min(config_.max_vel, state.lin_vel + config_.max_lin_acc * config_.dt);
        window.w_min = std::max(-config_.max_w, state.ang_vel - config_.max_ang_acc * config_.dt);
        window.w_max = std::min(config_.max_w, state.ang_vel + config_.max_ang_acc * config_.dt);
        return window;
    }

     std::vector<control> GenerateVelocitySamples(const DynamicWindow& window){
        std::vector<control> samples;
        double v_step = (config_.v_samples > 1)
            ? (window.v_max - window.v_min) / (config_.v_samples - 1)
            : 0.0;
        double w_step = (config_.w_samples > 1)
            ? (window.w_max - window.w_min) / (config_.w_samples - 1)
            : 0.0;
        for(int i=0; i < config_.v_samples; ++i){
            double v = window.v_min + i * v_step;
            for(int j=0; j< config_.w_samples; ++j){
                double w = window.w_min + j * w_step;
                samples.push_back({v, w});
            }
        }
        return samples;
    }

    std::vector<RobotState> SimulateTrajectory(const RobotState& initial_state,
         double v, double w){
        std :: vector<RobotState> trajectory;
        RobotState state = initial_state;
        double time = 0.0;
        while (time <= config_.predict_time){
            trajectory.push_back(state);
            state.x += v * std::cos(state.theta) * config_.dt;
            state.y += v * std::sin(state.theta) * config_.dt;
            state.theta += w * config_.dt;
            time += config_.dt;
        }       
        return trajectory;
    }

    bool IsOccupied(double x, double y){
        for(const auto& obs : obstacles_){
            double dx = x - obs.x;
            double dy = y - obs.y;
            if(std::sqrt(dx*dx + dy*dy) < robot_radius_){
                return true;
            }  
        }
        return false;
    }

    double ComputeScore(const std::vector<RobotState>& traj, double v,
                        const RobotState& current_state,
                        const RobotState& goal_state){
        if(traj.empty()) return -std::numeric_limits<double>::infinity();

        const RobotState& last_state = traj.back();
        const double dx = goal_state.x - last_state.x;
        const double dy = goal_state.y - last_state.y;
        const double goal_theta = std::atan2(dy, dx);
        double angle_diff = std::fabs(goal_theta - last_state.theta);
        if(angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

        const double start_dx = goal_state.x - current_state.x;
        const double start_dy = goal_state.y - current_state.y;
        const double start_dist = std::sqrt(start_dx * start_dx + start_dy * start_dy);
        const double end_dist = std::sqrt(dx * dx + dy * dy);

        double min_dist = std::numeric_limits<double>::infinity();
        for(const auto& state : traj){
            for(const auto& obs : obstacles_){
                const double ox = state.x - obs.x;
                const double oy = state.y - obs.y;
                const double dist = std::sqrt(ox * ox + oy * oy);
                if(dist < min_dist){
                    min_dist = dist;
                }
            }
        }

        const double safety_margin = 0.05;
        if(min_dist <= robot_radius_ + safety_margin){
            return -std::numeric_limits<double>::infinity();
        }

        if(!std::isfinite(min_dist)){
            min_dist = 2.0;
        }

        const double heading_score = 1.0 - Normalisation(angle_diff / M_PI);
        const double clearance_score = Normalisation(min_dist / 1.2);
        const double speed_score = Normalisation(v / std::max(0.05, config_.max_vel));
        const double progress_score = Normalisation((start_dist - end_dist) /
            std::max(0.1, config_.max_vel * config_.predict_time));

        double stop_penalty = 0.0;
        if(start_dist > 0.4 && v < 0.04){
            stop_penalty = 0.15;
        }

        return
            0.25 * heading_score +
            0.30 * clearance_score +
            0.20 * speed_score +
            0.25 * progress_score -
            stop_penalty;
    }

    PlanResult SelectBestVelocity(const RobotState& current_state, 
        const RobotState& goal_state){       
        DynamicWindow window = ComputeDynamicWindow(current_state);
        auto velocity_samples = GenerateVelocitySamples(window);
        PlanResult result;
        double best_score = -std::numeric_limits<double>::infinity();
        control best_control = {0.0, 0.0};

        const double gx = goal_state.x - current_state.x;
        const double gy = goal_state.y - current_state.y;
        const double dist_to_goal = std::sqrt(gx * gx + gy * gy);

        for(const auto& sample : velocity_samples){
            if(dist_to_goal > 0.4 && sample.v < 0.04){
                continue;
            }
            auto traj = SimulateTrajectory(current_state, sample.v, sample.w);
            result.sampled_trajs.push_back(traj);
            const double score = ComputeScore(traj, sample.v, current_state, goal_state);
            if(score > best_score){
                best_score = score;
                best_control = sample;
                result.best_traj = traj;
            }
        }
        result.best_control = best_control;
        result.valid = std::isfinite(best_score);
        return result;
    }    
};

class DWAPlannerNode : public rclcpp::Node {
public:
    DWAPlannerNode() : Node("dwa_planner_node"), planner_(makeConfig())
{   
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DWAPlannerNode::OdomCallback, this, std::placeholders::_1));

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DWAPlannerNode::ScanCallback, this, std::placeholders::_1));

        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&DWAPlannerNode::Goalcallback, this, std::placeholders::_1)); 
        goal_subscription_legacy_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10, std::bind(&DWAPlannerNode::Goalcallback, this, std::placeholders::_1)); 

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_markers", 10);


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DWAPlannerNode::ControlLoop, this)
        );

        planner_.setRobotRadius(robot_radius_);
        RCLCPP_INFO(this->get_logger(), "simple_dwa node initialized");

    }

private:

    static DWAConfig makeConfig(){
        DWAConfig cfg;
        cfg.max_vel = 0.24;
        cfg.max_w = 1.5;
        cfg.max_lin_acc = 0.5;
        cfg.max_ang_acc = 1.3;
        cfg.dt = 0.1;
        cfg.predict_time = 1.0;
        cfg.v_samples = 10;
        cfg.w_samples = 20;
        return cfg;
    }

    RobotState current_state_;
    geometry_msgs::msg::PoseStamped goal_;
    std::vector<Obstacle> obstacles_;
    double robot_radius_ = 0.25;
    bool has_odom_{false}; 
    bool has_goal_{false}; 
    bool goal_reached_logged_{false};
    double goal_tolerance_{0.15}; 
    bool has_last_cmd_{false};
    geometry_msgs::msg::Twist last_cmd_;
    int recovery_steps_{0};

    double safety_stop_dist_{0.24};
    double safety_slow_dist_{0.55};
    double safety_turn_speed_{1.4};

    DWAPlanner planner_;
   
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        current_state_.theta = YawFromQuaternionMsg(msg->pose.pose.orientation);
        
        current_state_.lin_vel = msg->twist.twist.linear.x;
        current_state_.ang_vel = msg->twist.twist.angular.z;
        has_odom_ = true; 
    }

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        obstacles_.clear();
        double angle = msg->angle_min;
        for(const auto& range : msg->ranges){
            if(std::isfinite(range) && range >= std::max(msg->range_min, 0.12f) && range <= msg->range_max){
                Obstacle obs;
                obs.x = current_state_.x +range * cos(current_state_.theta + angle);
                obs.y = current_state_.y +range * sin(current_state_.theta + angle);
                obstacles_.push_back(obs);
            }
            angle += msg->angle_increment;
        }
        planner_.updateObstacles(obstacles_);
    }   

    void Goalcallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        goal_ = *msg;     
        has_goal_ = true; 
        goal_reached_logged_ = false;
    }

    void ControlLoop(){
        
        if(!has_odom_ || !has_goal_){
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
            return;
        }

        RobotState goal_state;
        goal_state.x = goal_.pose.position.x;
        goal_state.y = goal_.pose.position.y;
        goal_state.theta = YawFromQuaternionMsg(goal_.pose.orientation);

        const double dx_goal = goal_state.x - current_state_.x;
        const double dy_goal = goal_state.y - current_state_.y;
        const double dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
        if(dist_to_goal <= goal_tolerance_){
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
            if(!goal_reached_logged_){
                RCLCPP_INFO(this->get_logger(), "Goal reached");
                goal_reached_logged_ = true;
            }
            return;
        }

        auto plan_result = planner_.SelectBestVelocity(current_state_, goal_state);
        PublishMarkers(plan_result.sampled_trajs, plan_result.best_traj, goal_state);
        geometry_msgs::msg::Twist cmd;
        if(!plan_result.valid){
            recovery_steps_++;
            const bool turn_left = SideClearance(true) >= SideClearance(false);
            if(recovery_steps_ <= 12){
                cmd.linear.x = 0.04;
                cmd.angular.z = turn_left ? 1.2 : -1.2;
            } 
            else {
                cmd.linear.x = 0.04;
                cmd.angular.z = turn_left ? 1.2 : -1.2;
            }
        } else {
            recovery_steps_ = 0;
            cmd.linear.x = plan_result.best_control.v;
            cmd.angular.z = plan_result.best_control.w;
        }

        cmd = SafetyFilter(cmd);

        if(!has_last_cmd_){
            last_cmd_ = cmd;
            has_last_cmd_ = true;
        } else {
            cmd.linear.x = 0.5 * cmd.linear.x + 0.5 * last_cmd_.linear.x;
            cmd.angular.z = 0.5 * cmd.angular.z + 0.5 * last_cmd_.angular.z;
            last_cmd_ = cmd;
        }

        cmd.linear.x = NormalisationVal(cmd.linear.x, -0.12, 0.24);
        cmd.angular.z = NormalisationVal(cmd.angular.z, -1.8, 1.8);
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Command v=%.2f w=%.2f",
            cmd.linear.x,
            cmd.angular.z);
    }

    double YawFromQuaternionMsg(const geometry_msgs::msg::Quaternion & q_msg){
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double NormalisationVal(double x, double lo, double hi){
        return std::max(lo, std::min(hi, x));
    }

    double FrontMinRange() const{
        if(obstacles_.empty()){
            return std::numeric_limits<double>::infinity();
        }

        double best = std::numeric_limits<double>::infinity();
        for(const auto& obs : obstacles_){
            const double dx = obs.x - current_state_.x;
            const double dy = obs.y - current_state_.y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            double ang = std::atan2(dy, dx) - current_state_.theta;
            while(ang > M_PI) ang -= 2.0 * M_PI;
            while(ang < -M_PI) ang += 2.0 * M_PI;
            if(std::fabs(ang) <= 0.52 && dist < best){
                best = dist;
            }
        }
        return best;
    }

    double SideClearance(bool left_side) const{
        if(obstacles_.empty()){
            return std::numeric_limits<double>::infinity();
        }

        double best = std::numeric_limits<double>::infinity();
        for(const auto& obs : obstacles_){
            const double dx = obs.x - current_state_.x;
            const double dy = obs.y - current_state_.y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            double ang = std::atan2(dy, dx) - current_state_.theta;
            while(ang > M_PI) ang -= 2.0 * M_PI;
            while(ang < -M_PI) ang += 2.0 * M_PI;

            const bool in_left = (ang >= 0.52 && ang <= 2.09);
            const bool in_right = (ang <= -0.52 && ang >= -2.09);
            if((left_side && in_left) || (!left_side && in_right)){
                if(dist < best){
                    best = dist;
                }
            }
        }
        return best;
    }

    geometry_msgs::msg::Twist SafetyFilter(const geometry_msgs::msg::Twist& cmd_in){
        geometry_msgs::msg::Twist cmd_out = cmd_in;
        const double d_front = FrontMinRange();

        if(d_front < safety_stop_dist_){
            const bool turn_left = SideClearance(true) >= SideClearance(false);
            cmd_out.linear.x = -0.06;
            cmd_out.angular.z = turn_left ? safety_turn_speed_ : -safety_turn_speed_;
            return cmd_out;
        }

        if(d_front < safety_slow_dist_){
            cmd_out.linear.x *= 0.7;
            if(std::fabs(cmd_out.angular.z) < 0.4){
                cmd_out.angular.z = (cmd_out.angular.z >= 0.0) ? 0.4 : -0.4;
            }
        }
        return cmd_out;
    }

    void PublishMarkers(const std::vector<std::vector<RobotState>>& all_trajs,
                        const std::vector<RobotState>& best_traj,
                        const RobotState& goal_state){
        visualization_msgs::msg::MarkerArray arr;
        const auto now = this->now();

        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = "odom";
        clear.header.stamp = now;
        clear.ns = "dwa_clear";
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);

        visualization_msgs::msg::Marker all;
        all.header.frame_id = "odom";
        all.header.stamp = now;
        all.ns = "all_trajectories";
        all.id = 1;
        all.type = visualization_msgs::msg::Marker::LINE_LIST;
        all.action = visualization_msgs::msg::Marker::ADD;
        all.scale.x = 0.02;
        all.color.r = 0.0f;
        all.color.g = 1.0f;
        all.color.b = 0.0f;
        all.color.a = 0.6f;
        for(const auto& traj : all_trajs){
            for(size_t i = 1; i < traj.size(); ++i){
                geometry_msgs::msg::Point p1;
                geometry_msgs::msg::Point p2;
                p1.x = traj[i - 1].x;
                p1.y = traj[i - 1].y;
                p1.z = 0.02;
                p2.x = traj[i].x;
                p2.y = traj[i].y;
                p2.z = 0.02;
                all.points.push_back(p1);
                all.points.push_back(p2);
            }
        }
        arr.markers.push_back(all);

        visualization_msgs::msg::Marker best;
        best.header.frame_id = "odom";
        best.header.stamp = now;
        best.ns = "best_trajectory";
        best.id = 2;
        best.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best.action = visualization_msgs::msg::Marker::ADD;
        best.scale.x = 0.06;
        best.color.r = 1.0f;
        best.color.g = 0.0f;
        best.color.b = 0.0f;
        best.color.a = 1.0f;
        for(const auto& p : best_traj){
            geometry_msgs::msg::Point q;
            q.x = p.x;
            q.y = p.y;
            q.z = 0.04;
            best.points.push_back(q);
        }
        arr.markers.push_back(best);

        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = "odom";
        goal_marker.header.stamp = now;
        goal_marker.ns = "goal_marker";
        goal_marker.id = 3;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position.x = goal_state.x;
        goal_marker.pose.position.y = goal_state.y;
        goal_marker.pose.position.z = 0.06;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.18;
        goal_marker.scale.y = 0.18;
        goal_marker.scale.z = 0.18;
        goal_marker.color.r = 1.0f;
        goal_marker.color.g = 0.0f;
        goal_marker.color.b = 0.0f;
        goal_marker.color.a = 1.0f;
        arr.markers.push_back(goal_marker);

        marker_pub_->publish(arr);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_legacy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWAPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
