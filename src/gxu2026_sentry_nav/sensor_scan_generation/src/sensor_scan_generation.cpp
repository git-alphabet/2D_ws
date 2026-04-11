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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <algorithm>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2/LinearMath/Matrix3x3.h"

namespace sensor_scan_generation
{

namespace
{

bool isNearIdentity(const tf2::Transform & t, const double eps = 1e-9)
{
  const auto & o = t.getOrigin();
  if (std::abs(o.x()) > eps || std::abs(o.y()) > eps || std::abs(o.z()) > eps) {
    return false;
  }

  const auto & q = t.getRotation();
  if (std::abs(q.x()) > eps || std::abs(q.y()) > eps || std::abs(q.z()) > eps) {
    return false;
  }
  // identity quaternion is (0,0,0,1)
  return std::abs(q.w() - 1.0) <= eps;
}

double yawFromQuat(const tf2::Quaternion & q)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<bool>("freeze_lidar_mount_tf", false);
  this->declare_parameter<bool>("debug_tf", false);
  this->declare_parameter<int>("debug_tf_throttle_ms", 1000);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.5);
  this->declare_parameter<bool>("fallback_to_latest_tf_on_extrapolation", true);
  this->declare_parameter<bool>("publish_base_tf", true);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("freeze_lidar_mount_tf", freeze_lidar_mount_tf_);
  this->get_parameter("debug_tf", debug_tf_);
  this->get_parameter("debug_tf_throttle_ms", debug_tf_throttle_ms_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  this->get_parameter(
    "fallback_to_latest_tf_on_extrapolation", fallback_to_latest_tf_on_extrapolation_);
  this->get_parameter("publish_base_tf", publish_base_tf_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;
  tf2::Transform tf_odom_to_odom_child;
  const rclcpp::Time odom_stamp = odometry_msg->header.stamp;
  const rclcpp::Time cloud_stamp = pcd_msg->header.stamp;
  const rclcpp::Time tf_lookup_stamp = std::min(odom_stamp, cloud_stamp);

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_odom_child);

  // 兼容 odin1 直连链路：若里程计 child_frame 不是 lidar_frame，则通过 TF 补齐 child->lidar。
  tf_odom_to_lidar = tf_odom_to_odom_child;
  const std::string odom_child_frame = odometry_msg->child_frame_id;
  if (odom_child_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "odometry.child_frame_id is empty, assume odometry pose already in lidar_frame='%s'",
      lidar_frame_.c_str());
  } else if (odom_child_frame != lidar_frame_) {
    const tf2::Transform tf_odom_child_to_lidar =
      getTransform(lidar_frame_, odom_child_frame, tf_lookup_stamp);
    tf_odom_to_lidar = tf_odom_to_odom_child * tf_odom_child_to_lidar;
  }

  if (freeze_lidar_mount_tf_ && !mount_tf_cached_) {
    try {
      const auto lidar_to_base = tf_buffer_->lookupTransform(
        lidar_frame_, base_frame_, tf_lookup_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      const auto lidar_to_robot_base = tf_buffer_->lookupTransform(
        lidar_frame_, robot_base_frame_, tf_lookup_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      tf2::fromMsg(lidar_to_base.transform, cached_lidar_to_base_);
      tf2::fromMsg(lidar_to_robot_base.transform, cached_lidar_to_robot_base_);
      mount_tf_cached_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "freeze_lidar_mount_tf=true: cached lidar mount TFs at t=%.3f (lidar='%s', base='%s', robot_base='%s')",
        tf_lookup_stamp.seconds(), lidar_frame_.c_str(), base_frame_.c_str(),
        robot_base_frame_.c_str());
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "freeze_lidar_mount_tf=true: waiting for TF (%s -> %s, %s -> %s): %s",
        base_frame_.c_str(), lidar_frame_.c_str(), robot_base_frame_.c_str(), lidar_frame_.c_str(),
        ex.what());
      return;
    }
  }

  if (freeze_lidar_mount_tf_) {
    tf_lidar_to_robot_base_ = cached_lidar_to_robot_base_;
    tf_lidar_to_chassis = cached_lidar_to_base_;
  } else {
    tf_lidar_to_robot_base_ = getTransform(lidar_frame_, robot_base_frame_, tf_lookup_stamp);
    tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, tf_lookup_stamp);
  }

  if (debug_tf_) {
    const tf2::Transform tf_chassis_to_robot_base = tf_lidar_to_chassis.inverse() * tf_lidar_to_robot_base_;
    const double yaw = yawFromQuat(tf_chassis_to_robot_base.getRotation());
    const double yaw_deg = yaw * 180.0 / M_PI;

    const bool id_lidar_to_chassis = isNearIdentity(tf_lidar_to_chassis);
    const bool id_lidar_to_robot_base = isNearIdentity(tf_lidar_to_robot_base_);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(),
      static_cast<uint64_t>(std::max(1, debug_tf_throttle_ms_)),
      "TF debug: got '%s'->'%s'=%s, '%s'->'%s'=%s; derived '%s'->'%s' yaw=%.6f rad (%.2f deg) at t=%.3f",
      base_frame_.c_str(), lidar_frame_.c_str(), id_lidar_to_chassis ? "IDENTITY" : "OK",
      robot_base_frame_.c_str(), lidar_frame_.c_str(), id_lidar_to_robot_base ? "IDENTITY" : "OK",
      base_frame_.c_str(), robot_base_frame_.c_str(), yaw, yaw_deg,
        tf_lookup_stamp.seconds());
  }

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  if (publish_base_tf_) {
    publishTransform(
      tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
  }
  publishOdometry(
    tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_, pcd_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (const tf2::ExtrapolationException & ex) {
    if (!fallback_to_latest_tf_on_extrapolation_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup extrapolation (%s -> %s) at t=%.3f: %s. Returning identity.",
        source_frame.c_str(), target_frame.c_str(), time.seconds(), ex.what());
      return tf2::Transform::getIdentity();
    }

    try {
      const rclcpp::Time latest_time(0, 0, this->get_clock()->get_clock_type());
      auto latest_transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, latest_time,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      tf2::Transform latest_transform;
      tf2::fromMsg(latest_transform_stamped.transform, latest_transform);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup extrapolation (%s -> %s) at t=%.3f: %s. Fallback to latest TF at t=%.3f.",
        source_frame.c_str(), target_frame.c_str(), time.seconds(), ex.what(),
        rclcpp::Time(latest_transform_stamped.header.stamp).seconds());
      return latest_transform;
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF extrapolation fallback failed (%s -> %s): %s. Returning identity.",
        source_frame.c_str(), target_frame.c_str(), latest_ex.what());
      return tf2::Transform::getIdentity();
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed (%s -> %s) at t=%.3f: %s. Returning identity.",
      source_frame.c_str(), target_frame.c_str(), time.seconds(), ex.what());
    return tf2::Transform::getIdentity();
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  static tf2::Transform previous_transform;
  static auto previous_time = std::chrono::steady_clock::now();
  const auto current_time = std::chrono::steady_clock::now();

  const double dt =
    std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time).count() *
    1e-9;

  if (dt > 0) {
    const auto linear_velocity = (transform.getOrigin() - previous_transform.getOrigin()) / dt;

    const tf2::Quaternion q_diff =
      transform.getRotation() * previous_transform.getRotation().inverse();
    const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  }

  previous_transform = transform;
  previous_time = current_time;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
