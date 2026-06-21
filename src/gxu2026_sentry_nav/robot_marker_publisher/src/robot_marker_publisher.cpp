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
  this->declare_parameter<double>("arrow_side", 0.45);
  this->declare_parameter<double>("robot_radius", 0.225);

  // Get parameters
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("marker_topic", marker_topic_);
  this->get_parameter("marker_color_r", marker_color_r_);
  this->get_parameter("marker_color_g", marker_color_g_);
  this->get_parameter("marker_color_b", marker_color_b_);
  this->get_parameter("arrow_side", arrow_side_);
  this->get_parameter("robot_radius", robot_radius_);

  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

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
  visualization_msgs::msg::MarkerArray array;
  auto stamp = msg->header.stamp;

  // Triangle marker (yaw direction indicator at footprint edge)
  visualization_msgs::msg::Marker triangle;
  triangle.header.frame_id = "odom";
  triangle.header.stamp = stamp;
  triangle.ns = "robot";
  triangle.id = 0;
  triangle.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  triangle.action = visualization_msgs::msg::Marker::ADD;

  // Offset centroid along local +x: base tangent to circle, tip pointing outward
  auto & q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  double tip_dist = arrow_side_ / std::sqrt(3.0);
  double half_base = arrow_side_ / (2.0 * std::sqrt(3.0));
  double offset = robot_radius_ + half_base;
  triangle.pose.position.x = msg->pose.pose.position.x + offset * std::cos(yaw);
  triangle.pose.position.y = msg->pose.pose.position.y + offset * std::sin(yaw);
  triangle.pose.position.z = msg->pose.pose.position.z;
  triangle.pose.orientation = msg->pose.pose.orientation;

  triangle.scale.x = 1.0;
  triangle.scale.y = 1.0;
  triangle.scale.z = 1.0;
  triangle.color.r = marker_color_r_;
  triangle.color.g = marker_color_g_;
  triangle.color.b = marker_color_b_;
  triangle.color.a = 0.8;

  // Equilateral triangle vertices: centroid at origin, tip pointing +x
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = tip_dist;  p1.y = 0.0;           p1.z = 0.0;
  p2.x = -half_base; p2.y = arrow_side_ / 2.0; p2.z = 0.0;
  p3.x = -half_base; p3.y = -arrow_side_ / 2.0; p3.z = 0.0;
  triangle.points.push_back(p1);
  triangle.points.push_back(p2);
  triangle.points.push_back(p3);
  array.markers.push_back(triangle);

  marker_pub_->publish(array);
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
