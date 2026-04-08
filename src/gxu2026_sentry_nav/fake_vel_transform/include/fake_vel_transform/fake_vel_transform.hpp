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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <limits>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/rmul.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void robotControlCallback(const sp_msgs::msg::RMUL::SharedPtr msg);
  void cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg);
  void manualSpinOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void publishTransform();
  void publishHoldCmdVelIfNeeded(const rclcpp::Time & now);
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sp_msgs::msg::RMUL>::SharedPtr robot_control_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_spin_override_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string robot_control_topic_;
  std::string cmd_spin_topic_;
  std::string manual_spin_override_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string angular_z_mode_;
  float spin_feedforward_base_speed_{0.0f};
  double controller_angular_z_scale_{1.0};
  double spin_feedforward_scale_{1.0};
  double spin_ff_velocity_decay_gain_{0.0};
  double angular_z_lower_limit_{-std::numeric_limits<double>::infinity()};
  double angular_z_upper_limit_{std::numeric_limits<double>::infinity()};
  double max_abs_angular_z_{0.0};
  float spin_speed_{0.0f};
  bool has_received_cmd_spin_{false};
  bool spin_enabled_{false};
  bool last_spin_enabled_logged_{false};
  bool use_manual_spin_override_{false};
  bool manual_spin_override_enabled_{false};
  bool disable_spin_while_moving_{true};
  bool cmd_spin_override_logged_{false};

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_;
  rclcpp::Time last_controller_activate_time_;

  rclcpp::Time last_cmd_vel_rx_time_;
  rclcpp::Time last_cmd_vel_pub_time_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
