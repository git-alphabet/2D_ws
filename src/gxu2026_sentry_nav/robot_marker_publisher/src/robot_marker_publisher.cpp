// Copyright 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robot_marker_publisher/robot_marker_publisher.hpp"

namespace robot_marker_publisher
{

RobotMarkerPublisher::RobotMarkerPublisher(const rclcpp::NodeOptions & options)
: Node("robot_marker_publisher", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("odometry_topic", "odin1/odometry");
  this->declare_parameter<std::string>("marker_topic", "robot_marker");
  this->declare_parameter<double>("marker_color_r", 0.0);
  this->declare_parameter<double>("marker_color_g", 1.0);
  this->declare_parameter<double>("marker_color_b", 0.0);
  this->declare_parameter<double>("marker_scale_x", 0.5);
  this->declare_parameter<double>("marker_scale_y", 0.1);
  this->declare_parameter<double>("marker_scale_z", 0.1);

  // Get parameters
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("marker_topic", marker_topic_);
  this->get_parameter("marker_color_r", marker_color_r_);
  this->get_parameter("marker_color_g", marker_color_g_);
  this->get_parameter("marker_color_b", marker_color_b_);
  this->get_parameter("marker_scale_x", marker_scale_x_);
  this->get_parameter("marker_scale_y", marker_scale_y_);
  this->get_parameter("marker_scale_z", marker_scale_z_);

  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

  // Subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 10,
    std::bind(&RobotMarkerPublisher::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Robot Marker Publisher started");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", odometry_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", marker_topic_.c_str());
}

void RobotMarkerPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto marker = visualization_msgs::msg::Marker();

  // Header
  marker.header.frame_id = msg->child_frame_id;
  marker.header.stamp = msg->header.stamp;

  // Marker properties
  marker.ns = "robot";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position (from odometry)
  marker.pose.position = msg->pose.pose.position;

  // Orientation (from odometry)
  marker.pose.orientation = msg->pose.pose.orientation;

  // Scale (length, width, height)
  marker.scale.x = marker_scale_x_;
  marker.scale.y = marker_scale_y_;
  marker.scale.z = marker_scale_z_;

  // Color
  marker.color.r = marker_color_r_;
  marker.color.g = marker_color_g_;
  marker.color.b = marker_color_b_;
  marker.color.a = 1.0;

  // Lifetime (0 = forever)
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;

  marker_pub_->publish(marker);
}

}  // namespace robot_marker_publisher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_marker_publisher::RobotMarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
