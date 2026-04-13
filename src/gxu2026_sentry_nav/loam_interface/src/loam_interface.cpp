// Copyright 2025 Lihan Chen
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

#include "loam_interface/loam_interface.hpp"

#include <algorithm>
#include <cctype>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

bool LoamInterfaceNode::updateBaseFrameToLidarTransform(
  const rclcpp::Time & stamp,
  bool use_latest_stamp,
  bool allow_latest_fallback)
{
  const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);

  auto try_lookup = [&](bool latest) -> bool {
    try {
      geometry_msgs::msg::TransformStamped tf_stamped;
      if (latest) {
        tf_stamped = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
      } else {
        tf_stamped = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, stamp, timeout);
      }

      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar_);
      base_frame_to_lidar_initialized_ = true;
      return true;
    } catch (tf2::TransformException & ex) {
      if (!latest && allow_latest_fallback) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "TF lookup (%s->%s @ %.3f) failed: %s, fallback to latest.",
          base_frame_.c_str(),
          lidar_frame_.c_str(),
          stamp.seconds(),
          ex.what());
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "TF lookup (%s->%s) failed: %s",
          base_frame_.c_str(),
          lidar_frame_.c_str(),
          ex.what());
      }
      return false;
    }
  };

  if (try_lookup(use_latest_stamp)) {
    return true;
  }

  if (!use_latest_stamp && allow_latest_fallback) {
    return try_lookup(true);
  }

  return false;
}

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  const auto to_lower = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  };

  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("input_odom_semantics", "lidar_odom_to_lidar");
  this->declare_parameter<std::string>("input_cloud_semantics", "lidar_odom");
  this->declare_parameter<bool>("freeze_base_to_lidar_tf", true);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.2);

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("input_odom_semantics", input_odom_semantics_);
  this->get_parameter("input_cloud_semantics", input_cloud_semantics_);
  this->get_parameter("freeze_base_to_lidar_tf", freeze_base_to_lidar_tf_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

  input_odom_semantics_ = to_lower(input_odom_semantics_);
  input_cloud_semantics_ = to_lower(input_cloud_semantics_);

  if (input_odom_semantics_ != "lidar_odom_to_lidar" && input_odom_semantics_ != "odom_to_base") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid input_odom_semantics='%s', fallback to 'lidar_odom_to_lidar'",
      input_odom_semantics_.c_str());
    input_odom_semantics_ = "lidar_odom_to_lidar";
  }

  if (input_cloud_semantics_ != "lidar_odom" && input_cloud_semantics_ != "odom") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid input_cloud_semantics='%s', fallback to 'lidar_odom'",
      input_cloud_semantics_.c_str());
    input_cloud_semantics_ = "lidar_odom";
  }

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "loam_interface semantics: input_odom_semantics=%s, input_cloud_semantics=%s, freeze_base_to_lidar_tf=%s, tf_lookup_timeout_sec=%.3f",
    input_odom_semantics_.c_str(),
    input_cloud_semantics_.c_str(),
    freeze_base_to_lidar_tf_ ? "true" : "false",
    tf_lookup_timeout_sec_);
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();

  if (input_cloud_semantics_ == "odom") {
    // odin1 cloud_slam already uses odom frame semantics; keep cloud untouched.
    *out = *msg;
    out->header.frame_id = odom_frame_;
  } else {
    const rclcpp::Time msg_stamp(msg->header.stamp);
    if (freeze_base_to_lidar_tf_) {
      if (!base_frame_to_lidar_initialized_) {
        if (!updateBaseFrameToLidarTransform(msg_stamp, true, false)) {
          return;
        }
      }
    } else {
      if (!updateBaseFrameToLidarTransform(msg_stamp, false, true)) {
        return;
      }
    }

    // Legacy path: input point cloud is based on lidar_odom and needs remap to odom.
    pcl_ros::transformPointCloud(odom_frame_, tf_base_frame_to_lidar_, *msg, *out);
  }

  pcd_pub_->publish(*out);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time msg_stamp(msg->header.stamp);

  if (freeze_base_to_lidar_tf_) {
    if (!base_frame_to_lidar_initialized_) {
      if (!updateBaseFrameToLidarTransform(msg_stamp, true, false)) {
        return;
      }
    }
  } else {
    if (!updateBaseFrameToLidarTransform(msg_stamp, false, true)) {
      return;
    }
  }

  tf2::Transform tf_input_odom_to_child;
  tf2::fromMsg(msg->pose.pose, tf_input_odom_to_child);

  tf2::Transform tf_odom_to_lidar;
  if (input_odom_semantics_ == "odom_to_base") {
    // odin1 path: input odometry is odom->base, output odom->lidar.
    tf_odom_to_lidar = tf_input_odom_to_child * tf_base_frame_to_lidar_;
  } else {
    // legacy path: input odometry is lidar_odom->lidar.
    tf_odom_to_lidar = tf_base_frame_to_lidar_ * tf_input_odom_to_child;
  }

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
