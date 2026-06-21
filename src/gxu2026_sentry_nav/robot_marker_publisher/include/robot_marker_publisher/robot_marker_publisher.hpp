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

#ifndef ROBOT_MARKER_PUBLISHER__ROBOT_MARKER_PUBLISHER_HPP_
#define ROBOT_MARKER_PUBLISHER__ROBOT_MARKER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>

namespace robot_marker_publisher
{

class RobotMarkerPublisher : public rclcpp::Node
{
public:
  explicit RobotMarkerPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Parameters
  std::string odometry_topic_;
  std::string marker_topic_;
  double marker_color_r_;
  double marker_color_g_;
  double marker_color_b_;
  double arrow_side_;
  double robot_radius_;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace robot_marker_publisher

#endif  // ROBOT_MARKER_PUBLISHER__ROBOT_MARKER_PUBLISHER_HPP_
