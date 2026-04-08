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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "example_interfaces/msg/float32.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;
constexpr double OUTPUT_HOLD_PUBLISH_TIMEOUT = 0.1;
constexpr double SPIN_LINEAR_STOP_THRESHOLD = 0.05;
constexpr const char * ANGULAR_Z_MODE_SPIN_ONLY = "spin_only";
constexpr const char * ANGULAR_Z_MODE_CONTROLLER_ONLY = "controller_only";
constexpr const char * ANGULAR_Z_MODE_CONTROLLER_PLUS_SPIN_FF = "controller_plus_spin_ff";
constexpr const char * ANGULAR_Z_MODE_BLEND_ALIAS = "blend";

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Initialize state
  current_robot_base_angle_ = 0.0;
  last_controller_activate_time_ = this->get_clock()->now();
  last_cmd_vel_rx_time_ = last_controller_activate_time_;
  last_cmd_vel_pub_time_ = last_controller_activate_time_;

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("robot_control_topic", "robot_control");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<bool>("use_manual_spin_override", false);
  this->declare_parameter<std::string>("manual_spin_override_topic", "manual_chassis_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("angular_z_mode", ANGULAR_Z_MODE_SPIN_ONLY);
  this->declare_parameter<double>(
    "spin_feedforward_base_speed", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<float>("init_spin_speed", 0.0);  // legacy alias
  this->declare_parameter<double>("controller_angular_z_scale", 1.0);
  this->declare_parameter<double>("spin_feedforward_scale", 1.0);
  this->declare_parameter<double>("spin_ff_velocity_decay_gain", 0.0);
  this->declare_parameter<double>(
    "angular_z_lower_limit", -std::numeric_limits<double>::infinity());
  this->declare_parameter<double>(
    "angular_z_upper_limit", std::numeric_limits<double>::infinity());
  this->declare_parameter<double>("max_abs_angular_z", 0.0);
  this->declare_parameter<bool>("disable_spin_while_moving", true);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("robot_control_topic", robot_control_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("use_manual_spin_override", use_manual_spin_override_);
  this->get_parameter("manual_spin_override_topic", manual_spin_override_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("angular_z_mode", angular_z_mode_);
  double spin_feedforward_base_speed_param = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter("spin_feedforward_base_speed", spin_feedforward_base_speed_param);
  this->get_parameter("init_spin_speed", spin_feedforward_base_speed_);
  if (std::isfinite(spin_feedforward_base_speed_param)) {
    spin_feedforward_base_speed_ = static_cast<float>(spin_feedforward_base_speed_param);
  }
  this->get_parameter("controller_angular_z_scale", controller_angular_z_scale_);
  this->get_parameter("spin_feedforward_scale", spin_feedforward_scale_);
  this->get_parameter("spin_ff_velocity_decay_gain", spin_ff_velocity_decay_gain_);
  this->get_parameter("angular_z_lower_limit", angular_z_lower_limit_);
  this->get_parameter("angular_z_upper_limit", angular_z_upper_limit_);
  this->get_parameter("max_abs_angular_z", max_abs_angular_z_);
  this->get_parameter("disable_spin_while_moving", disable_spin_while_moving_);

  if (angular_z_lower_limit_ > angular_z_upper_limit_) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid angular_z limits: lower(%.3f) > upper(%.3f), fallback to unbounded.",
      angular_z_lower_limit_, angular_z_upper_limit_);
    angular_z_lower_limit_ = -std::numeric_limits<double>::infinity();
    angular_z_upper_limit_ = std::numeric_limits<double>::infinity();
  }

  std::transform(
    angular_z_mode_.begin(), angular_z_mode_.end(), angular_z_mode_.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (angular_z_mode_ == ANGULAR_Z_MODE_BLEND_ALIAS) {
    angular_z_mode_ = ANGULAR_Z_MODE_CONTROLLER_PLUS_SPIN_FF;
  }
  if (
    angular_z_mode_ != ANGULAR_Z_MODE_SPIN_ONLY &&
    angular_z_mode_ != ANGULAR_Z_MODE_CONTROLLER_ONLY &&
    angular_z_mode_ != ANGULAR_Z_MODE_CONTROLLER_PLUS_SPIN_FF)
  {
    RCLCPP_WARN(
      get_logger(), "Invalid angular_z_mode='%s', fallback to '%s'", angular_z_mode_.c_str(),
      ANGULAR_Z_MODE_SPIN_ONLY);
    angular_z_mode_ = ANGULAR_Z_MODE_SPIN_ONLY;
  }

  RCLCPP_INFO(
    get_logger(),
    "Spin control: topic=%s, mode=%s, spin_ff_base=%.3f, ctrl_scale=%.3f, ff_scale=%.3f, ff_decay=%.3f, lower_w=%.3f, upper_w=%.3f, max_abs_w=%.3f, disable_spin_while_moving=%s",
    robot_control_topic_.c_str(), angular_z_mode_.c_str(), spin_feedforward_base_speed_,
    controller_angular_z_scale_, spin_feedforward_scale_,
    spin_ff_velocity_decay_gain_, angular_z_lower_limit_, angular_z_upper_limit_, max_abs_angular_z_,
    disable_spin_while_moving_ ? "true" : "false");

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  robot_control_sub_ = this->create_subscription<sp_msgs::msg::RMUL>(
    robot_control_topic_, 10,
    std::bind(&FakeVelTransform::robotControlCallback, this, std::placeholders::_1));
  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 10,
    std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  if (use_manual_spin_override_) {
    manual_spin_override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      manual_spin_override_topic_, 10,
      std::bind(&FakeVelTransform::manualSpinOverrideCallback, this, std::placeholders::_1));
    RCLCPP_WARN(
      get_logger(),
      "Manual spin override enabled: topic=%s (this overrides RMUL.chassis_spin)",
      manual_spin_override_topic_.c_str());
  }
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  // Use create_timer (sim-time aware) instead of create_wall_timer to avoid
  // non-monotonic TF timestamps when Gazebo sim-time pauses or fluctuates.
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(20),
    std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::robotControlCallback(const sp_msgs::msg::RMUL::SharedPtr msg)
{
  if (use_manual_spin_override_) {
    return;
  }

  const bool prev = spin_enabled_;
  spin_enabled_ = msg->chassis_spin;
  if (spin_enabled_ != prev || !last_spin_enabled_logged_) {
    RCLCPP_INFO(
      get_logger(), "Spin switch updated: chassis_spin=%s",
      spin_enabled_ ? "true" : "false");
    last_spin_enabled_logged_ = true;
  }
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
  has_received_cmd_spin_ = true;
  if (!cmd_spin_override_logged_) {
    RCLCPP_INFO(
      get_logger(),
      "Received cmd_spin for the first time, dynamic cmd_spin now overrides spin_feedforward_base_speed.");
    cmd_spin_override_logged_ = true;
  }
}

void FakeVelTransform::manualSpinOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool prev = manual_spin_override_enabled_;
  manual_spin_override_enabled_ = msg->data;

  if (manual_spin_override_enabled_ != prev || !last_spin_enabled_logged_) {
    RCLCPP_INFO(
      get_logger(), "Manual spin override updated: enabled=%s",
      manual_spin_override_enabled_ ? "true" : "false");
    last_spin_enabled_logged_ = true;
  }
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  if ((this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const auto now = this->get_clock()->now();
  last_cmd_vel_rx_time_ = now;
  latest_cmd_vel_ = msg;
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
    last_cmd_vel_pub_time_ = now;
  } else {
    // Keep latest_cmd_vel_ for sync callback
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->get_clock()->now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  last_cmd_vel_pub_time_ = this->get_clock()->now();
}

void FakeVelTransform::publishTransform()
{
  const auto now = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  publishHoldCmdVelIfNeeded(now);
}

void FakeVelTransform::publishHoldCmdVelIfNeeded(const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);

  // If output cmd_vel is still flowing, do nothing.
  if ((now - last_cmd_vel_pub_time_).seconds() <= OUTPUT_HOLD_PUBLISH_TIMEOUT) {
    return;
  }

  geometry_msgs::msg::Twist base_cmd;

  // If upstream cmd_vel becomes stale, force linear/ang.z base to 0 to avoid runaway.
  const bool upstream_stale = (now - last_cmd_vel_rx_time_).seconds() > CONTROLLER_TIMEOUT;
  if (!upstream_stale && latest_cmd_vel_) {
    base_cmd = *latest_cmd_vel_;
  } else {
    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    base_cmd.linear.z = 0.0;
    base_cmd.angular.x = 0.0;
    base_cmd.angular.y = 0.0;
    base_cmd.angular.z = 0.0;
  }

  auto base_cmd_ptr = std::make_shared<geometry_msgs::msg::Twist>(base_cmd);
  auto aft_tf_vel = transformVelocity(base_cmd_ptr, current_robot_base_angle_);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  last_cmd_vel_pub_time_ = now;
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;

  float current_spin = 0.0f;
  // Always apply cmd_spin value; spin on/off is controlled by NonlinearSpinPublisher
  // which publishes 0 when chassis_spin=False via BT RobotControl.
  if (has_received_cmd_spin_) {
    current_spin = spin_speed_;
  } else {
    current_spin = spin_feedforward_base_speed_;
  }

  const double linear_speed = std::hypot(twist->linear.x, twist->linear.y);
  const bool is_moving = linear_speed > SPIN_LINEAR_STOP_THRESHOLD;
  const double controller_angular_z = twist->angular.z * controller_angular_z_scale_;

  double spin_feedforward = static_cast<double>(current_spin) * spin_feedforward_scale_;
  if (spin_ff_velocity_decay_gain_ > 0.0) {
    spin_feedforward /= (1.0 + spin_ff_velocity_decay_gain_ * linear_speed);
  }
  if (disable_spin_while_moving_ && is_moving) {
    spin_feedforward = 0.0;
  }

  if (angular_z_mode_ == ANGULAR_Z_MODE_CONTROLLER_ONLY) {
    aft_tf_vel.angular.z = controller_angular_z;
  } else if (angular_z_mode_ == ANGULAR_Z_MODE_CONTROLLER_PLUS_SPIN_FF) {
    aft_tf_vel.angular.z = controller_angular_z + spin_feedforward;
  } else {
    aft_tf_vel.angular.z = spin_feedforward;
  }

  aft_tf_vel.angular.z = std::clamp(
    aft_tf_vel.angular.z, angular_z_lower_limit_, angular_z_upper_limit_);

  if (max_abs_angular_z_ > 0.0) {
    aft_tf_vel.angular.z = std::clamp(aft_tf_vel.angular.z, -max_abs_angular_z_, max_abs_angular_z_);
  }

  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
