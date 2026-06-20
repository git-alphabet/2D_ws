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
  this->declare_parameter<double>("dot_radius", 0.15);

  // Get parameters
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("marker_topic", marker_topic_);
  this->get_parameter("marker_color_r", marker_color_r_);
  this->get_parameter("marker_color_g", marker_color_g_);
  this->get_parameter("marker_color_b", marker_color_b_);
  this->get_parameter("marker_scale_x", marker_scale_x_);
  this->get_parameter("marker_scale_y", marker_scale_y_);
  this->get_parameter("marker_scale_z", marker_scale_z_);
  this->get_parameter("dot_radius", dot_radius_);

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

  // Triangle marker (yaw direction indicator)
  visualization_msgs::msg::Marker triangle;
  triangle.header.frame_id = "odom";
  triangle.header.stamp = stamp;
  triangle.ns = "robot";
  triangle.id = 0;
  triangle.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  triangle.action = visualization_msgs::msg::Marker::ADD;
  triangle.pose.position = msg->pose.pose.position;
  triangle.pose.orientation = msg->pose.pose.orientation;
  triangle.scale.x = 1.0;
  triangle.scale.y = 1.0;
  triangle.scale.z = 1.0;
  triangle.color.r = marker_color_r_;
  triangle.color.g = marker_color_g_;
  triangle.color.b = marker_color_b_;
  triangle.color.a = 0.8;

  // Triangle vertices: arrow pointing +x, centered at origin
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = marker_scale_x_; p1.y = 0.0; p1.z = 0.0;
  p2.x = -marker_scale_x_ * 0.5; p2.y = marker_scale_y_ * 0.5; p2.z = 0.0;
  p3.x = -marker_scale_x_ * 0.5; p3.y = -marker_scale_y_ * 0.5; p3.z = 0.0;
  triangle.points.push_back(p1);
  triangle.points.push_back(p2);
  triangle.points.push_back(p3);
  array.markers.push_back(triangle);

  // Dot marker (center position indicator)
  visualization_msgs::msg::Marker dot;
  dot.header.frame_id = "odom";
  dot.header.stamp = stamp;
  dot.ns = "robot";
  dot.id = 1;
  dot.type = visualization_msgs::msg::Marker::SPHERE;
  dot.action = visualization_msgs::msg::Marker::ADD;
  dot.pose.position = msg->pose.pose.position;
  dot.pose.orientation.w = 1.0;
  dot.scale.x = dot_radius_ * 2;
  dot.scale.y = dot_radius_ * 2;
  dot.scale.z = dot_radius_ * 2;
  dot.color.r = 1.0;
  dot.color.g = 1.0;
  dot.color.b = 1.0;
  dot.color.a = 1.0;
  array.markers.push_back(dot);

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
