// Copyright 2026 iRobot Corporation. All Rights Reserved.

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace shampy_nodes
{

struct RobotState
{
  tf2::Transform pose;
  sensor_msgs::msg::LaserScan last_scan;
};

class MotionControlNode : public rclcpp::Node
{
public:
  explicit MotionControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("motion_control", options)
  {
    // Parameters
    this->declare_parameter<double>("max_speed", 0.306);
    this->declare_parameter<double>("safety_distance", 0.5); // Minimum distance to obstacle

    // Subscriptions
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&MotionControlNode::scan_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&MotionControlNode::odom_callback, this, std::placeholders::_1));

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&MotionControlNode::teleop_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "diffdrive_controller/cmd_vel", rclcpp::SystemDefaultsQoS());

    wheel_status_pub_ = this->create_publisher<irobot_create_msgs::msg::WheelStatus>(
      "wheel_status", rclcpp::SensorDataQoS().reliable());

    // Services
    e_stop_server_ = this->create_service<irobot_create_msgs::srv::EStop>(
      "e_stop",
      std::bind(&MotionControlNode::e_stop_request, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for control loop (40Hz)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(25),
      std::bind(&MotionControlNode::control_robot, this));

    RCLCPP_INFO(this->get_logger(), "New Motion Control Node started with LiDAR support.");
  }

private:
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.last_scan = *msg;
    
    // Simple obstacle detection
    bool obstacle_detected = false;
    double min_dist = this->get_parameter("safety_distance").as_double();
    for (float range : msg->ranges) {
      if (range < min_dist && range > msg->range_min) {
        obstacle_detected = true;
        break;
      }
    }
    obstacle_near_ = obstacle_detected;
  }

  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    tf2::fromMsg(msg->pose.pose, current_state_.pose);
  }

  void teleop_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_teleop_cmd_ = *msg;
    last_teleop_time_ = this->now();
  }

  void e_stop_request(
    const irobot_create_msgs::srv::EStop::Request::SharedPtr request,
    irobot_create_msgs::srv::EStop::Response::SharedPtr response)
  {
    e_stop_engaged_ = request->e_stop_on;
    response->success = true;
    response->message = e_stop_engaged_ ? "E-Stop engaged" : "E-Stop disengaged";
    RCLCPP_INFO(this->get_logger(), "E-Stop request: %s", response->message.c_str());
  }

  void control_robot()
  {
    geometry_msgs::msg::Twist cmd_out;

    std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
    
    // Check for teleop timeout (0.5s)
    if ((this->now() - last_teleop_time_) < rclcpp::Duration::from_seconds(0.5)) {
      cmd_out = last_teleop_cmd_;
    } else {
      // Stop if no recent command
      cmd_out.linear.x = 0.0;
      cmd_out.angular.z = 0.0;
    }

    // Apply safety limits based on LiDAR
    if (obstacle_near_ && cmd_out.linear.x > 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Obstacle detected! Stopping forward motion.");
      cmd_out.linear.x = 0.0;
    }

    // Apply max speed limit
    double max_speed = this->get_parameter("max_speed").as_double();
    if (std::abs(cmd_out.linear.x) > max_speed) {
      cmd_out.linear.x = std::copysign(max_speed, cmd_out.linear.x);
    }

    // If E-Stop is engaged, zero everything
    if (e_stop_engaged_) {
      cmd_out.linear.x = 0.0;
      cmd_out.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd_out);

    // Publish wheel status
    auto wheel_status_msg = std::make_unique<irobot_create_msgs::msg::WheelStatus>();
    wheel_status_msg->header.stamp = this->now();
    wheel_status_msg->header.frame_id = "base_link";
    wheel_status_msg->wheels_enabled = !e_stop_engaged_;
    wheel_status_pub_->publish(std::move(wheel_status_msg));
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<irobot_create_msgs::msg::WheelStatus>::SharedPtr wheel_status_pub_;

  // Services
  rclcpp::Service<irobot_create_msgs::srv::EStop>::SharedPtr e_stop_server_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  RobotState current_state_;
  std::mutex state_mutex_;
  
  geometry_msgs::msg::Twist last_teleop_cmd_;
  rclcpp::Time last_teleop_time_{0, 0, RCL_ROS_TIME};
  std::mutex cmd_mutex_;

  std::atomic<bool> obstacle_near_{false};
  std::atomic<bool> e_stop_engaged_{false};
};

}  // namespace shampy_nodes

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(shampy_nodes::MotionControlNode)
