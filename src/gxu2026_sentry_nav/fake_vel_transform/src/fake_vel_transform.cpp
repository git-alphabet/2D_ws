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
#include <cmath>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;
constexpr double OUTPUT_HOLD_PUBLISH_TIMEOUT = 0.1;
constexpr double SPIN_LINEAR_STOP_THRESHOLD = 0.05;

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
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<bool>("disable_spin_while_moving", true);
  this->declare_parameter<bool>("enable_speed_bump_min_speed", true);
  this->declare_parameter<double>("speed_bump_min_linear_speed", 1.5);
  this->declare_parameter<std::string>("speed_bump_map_frame", "map");
  this->declare_parameter<std::string>("speed_bump_zone_name", "speed_bump");
  this->declare_parameter<std::string>("speed_bump_zones_file", "");

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
  this->get_parameter("init_spin_speed", init_spin_speed_);
  this->get_parameter("disable_spin_while_moving", disable_spin_while_moving_);
  this->get_parameter("enable_speed_bump_min_speed", enable_speed_bump_min_speed_);
  this->get_parameter("speed_bump_min_linear_speed", speed_bump_min_linear_speed_);
  this->get_parameter("speed_bump_map_frame", speed_bump_map_frame_);
  this->get_parameter("speed_bump_zone_name", speed_bump_zone_name_);
  this->get_parameter("speed_bump_zones_file", speed_bump_zones_file_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  loadSpeedBumpZoneFromYaml();

  RCLCPP_INFO(
    get_logger(),
    "Spin control: topic=%s, init_spin_speed=%.3f rad/s, disable_spin_while_moving=%s",
    robot_control_topic_.c_str(), init_spin_speed_,
    disable_spin_while_moving_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Speed bump min speed: enabled=%s, min_linear_speed=%.3f, zone=%s, loaded=%s",
    enable_speed_bump_min_speed_ ? "true" : "false", speed_bump_min_linear_speed_,
    speed_bump_zone_name_.c_str(), speed_bump_zone_loaded_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Speed bump reference frame: %s",
    speed_bump_map_frame_.c_str());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  robot_control_sub_ = this->create_subscription<sp_msgs::msg::RMUCRobotControl>(
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

void FakeVelTransform::robotControlCallback(const sp_msgs::msg::RMUCRobotControl::SharedPtr msg)
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
      "Received cmd_spin for the first time, dynamic cmd_spin now overrides init_spin_speed.");
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
  updateRobotPositionInMapFrame(msg);

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

  updateRobotPositionInMapFrame(odom_msg);

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
  // spin_enabled_ 由 BT RmucRobotControl 的 chassis_spin 控制
  if (spin_enabled_) {
    if (has_received_cmd_spin_) {
      current_spin = spin_speed_;      // NonlinearSpinPublisher 动态值优先
    } else {
      current_spin = init_spin_speed_; // 回退到初始自旋速度
    }
  }

  const double linear_speed = std::hypot(twist->linear.x, twist->linear.y);
  const bool is_moving = linear_speed > SPIN_LINEAR_STOP_THRESHOLD;
  if (disable_spin_while_moving_ && is_moving) {
    aft_tf_vel.angular.z = 0.0;
  } else {
    aft_tf_vel.angular.z = current_spin;
  }

  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);

  if (enable_speed_bump_min_speed_ && speed_bump_zone_loaded_) {
    double robot_x;
    double robot_y;
    bool robot_pose_ready;
    {
      std::lock_guard<std::mutex> pose_lock(pose_mutex_);
      robot_x = current_robot_x_;
      robot_y = current_robot_y_;
      robot_pose_ready = robot_pose_in_map_ready_;
    }

    if (!robot_pose_ready) {
      return aft_tf_vel;
    }

    const bool in_speed_bump = pointInPolygon(robot_x, robot_y, speed_bump_zone_vertices_);
    if (in_speed_bump != was_in_speed_bump_zone_) {
      RCLCPP_INFO(
        get_logger(), "Speed bump zone state changed: in_zone=%s at (%.3f, %.3f)",
        in_speed_bump ? "true" : "false", robot_x, robot_y);
      was_in_speed_bump_zone_ = in_speed_bump;
    }

    if (in_speed_bump) {
      const double transformed_linear_speed = std::hypot(aft_tf_vel.linear.x, aft_tf_vel.linear.y);
      if (
        transformed_linear_speed > EPSILON &&
        transformed_linear_speed < speed_bump_min_linear_speed_)
      {
        const double scale = speed_bump_min_linear_speed_ / transformed_linear_speed;
        aft_tf_vel.linear.x *= scale;
        aft_tf_vel.linear.y *= scale;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Speed bump min speed applied: %.3f -> %.3f m/s",
          transformed_linear_speed, speed_bump_min_linear_speed_);
      }
    }
  }

  return aft_tf_vel;
}

void FakeVelTransform::updateRobotPositionInMapFrame(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const std::string src_frame = msg->header.frame_id;
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;

  if (!speed_bump_map_frame_.empty() && src_frame != speed_bump_map_frame_) {
    geometry_msgs::msg::PointStamped in_point;
    geometry_msgs::msg::PointStamped out_point;
    in_point.header = msg->header;
    in_point.point.x = msg->pose.pose.position.x;
    in_point.point.y = msg->pose.pose.position.y;
    in_point.point.z = msg->pose.pose.position.z;

    try {
      out_point = tf_buffer_->transform(in_point, speed_bump_map_frame_, tf2::durationFromSec(0.05));
      robot_x = out_point.point.x;
      robot_y = out_point.point.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform odometry pose from %s to %s: %s",
        src_frame.c_str(), speed_bump_map_frame_.c_str(), ex.what());
      return;
    }
  }

  {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    current_robot_x_ = robot_x;
    current_robot_y_ = robot_y;
    robot_pose_in_map_ready_ = true;
  }
}

void FakeVelTransform::loadSpeedBumpZoneFromYaml()
{
  speed_bump_zone_loaded_ = false;
  speed_bump_zone_vertices_.clear();

  if (!enable_speed_bump_min_speed_) {
    return;
  }

  if (speed_bump_zones_file_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Speed bump min speed enabled but speed_bump_zones_file is empty, feature disabled.");
    return;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(speed_bump_zones_file_);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load speed bump zones file %s: %s",
      speed_bump_zones_file_.c_str(), e.what());
    return;
  }

  const auto zones = config["zones"];
  if (!zones || !zones.IsSequence()) {
    RCLCPP_ERROR(
      get_logger(), "Invalid speed bump zones file %s: missing zones sequence",
      speed_bump_zones_file_.c_str());
    return;
  }

  for (const auto & zone : zones) {
    if (!zone["name"] || !zone["vertices"]) {
      continue;
    }

    const std::string name = zone["name"].as<std::string>();
    const std::string type = zone["type"] ? zone["type"].as<std::string>() : "";
    if (name != speed_bump_zone_name_) {
      continue;
    }
    if (!type.empty() && type != "speed_bump") {
      RCLCPP_WARN(
        get_logger(),
        "Speed bump zone %s has type %s, expected speed_bump, ignore this zone.",
        name.c_str(), type.c_str());
      continue;
    }

    std::vector<std::pair<double, double>> vertices;
    for (const auto & v : zone["vertices"]) {
      if (!v.IsSequence() || v.size() < 2) {
        continue;
      }
      vertices.emplace_back(v[0].as<double>(), v[1].as<double>());
    }

    if (vertices.size() < 3) {
      RCLCPP_WARN(
        get_logger(), "Speed bump zone %s has fewer than 3 vertices, ignore.",
        name.c_str());
      continue;
    }

    speed_bump_zone_vertices_ = std::move(vertices);
    speed_bump_zone_loaded_ = true;
    break;
  }

  if (!speed_bump_zone_loaded_) {
    RCLCPP_WARN(
      get_logger(),
      "Speed bump zone '%s' not found in %s, feature disabled.",
      speed_bump_zone_name_.c_str(), speed_bump_zones_file_.c_str());
  }
}

bool FakeVelTransform::pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & poly) const
{
  if (poly.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double xi = poly[i].first;
    const double yi = poly[i].second;
    const double xj = poly[j].first;
    const double yj = poly[j].second;
    if (((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
