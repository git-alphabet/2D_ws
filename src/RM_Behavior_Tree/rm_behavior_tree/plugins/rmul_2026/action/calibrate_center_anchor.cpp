#include "rm_behavior_tree/plugins/rmul_2026/action/calibrate_center_anchor.hpp"

#include <algorithm>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

CalibrateCenterAnchorAction::CalibrateCenterAnchorAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh ? params.nh : std::make_shared<rclcpp::Node>("calibrate_center_anchor");
  tf2::Duration buffer_duration(tf2::durationFromSec(10.0));
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), buffer_duration, node_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Automatically subscribe to literal point_topic early so we don't miss clicks during initialization
  auto r_topic = getInput<std::string>("point_topic");
  if (r_topic && !r_topic.value().empty() && r_topic.value().find("{") == std::string::npos) {
    updatePointSubscription(r_topic.value());
  }

}

void CalibrateCenterAnchorAction::publishOutputs(const std::string & map_frame)
{
  setOutput("center_x", center_x_);
  setOutput("center_y", center_y_);
  setOutput("center_yaw", center_yaw_);
  setOutput("center_valid", calibrated_);

  geometry_msgs::msg::PoseStamped center_pose;
  center_pose.header.frame_id = map_frame;
  center_pose.header.stamp = node_->get_clock()->now();
  center_pose.pose.position.x = center_x_;
  center_pose.pose.position.y = center_y_;
  center_pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, center_yaw_);
  center_pose.pose.orientation.x = q.x();
  center_pose.pose.orientation.y = q.y();
  center_pose.pose.orientation.z = q.z();
  center_pose.pose.orientation.w = q.w();
  setOutput("center_pose", center_pose);
}

void CalibrateCenterAnchorAction::updatePointSubscription(const std::string & point_topic)
{
  if (point_topic.empty()) {
    point_sub_.reset();
    point_topic_.clear();
    return;
  }

  if (point_sub_ && point_topic_ == point_topic) {
    return;
  }

  point_topic_ = point_topic;
  point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    point_topic_, 10,
    [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(point_mutex_);
      latest_point_ = *msg;
    });
}

void CalibrateCenterAnchorAction::publishCrossMarker(
  const std::string & map_frame,
  const std::string & marker_namespace,
  const std::string & marker_color,
  int marker_id,
  double marker_size)
{
  std::string color = marker_color;
  std::transform(color.begin(), color.end(), color.begin(), ::tolower);

  double r = 1.0;
  double g = 0.0;
  double b = 0.0;
  if (color == "black") {
    r = 0.0;
    g = 0.0;
    b = 0.0;
  }

  if (marker_size < 0.05) {
    marker_size = 0.05;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame;
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.08;
  marker.color.a = 1.0;
  marker.color.r = static_cast<float>(r);
  marker.color.g = static_cast<float>(g);
  marker.color.b = static_cast<float>(b);

  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;
  geometry_msgs::msg::Point p3;
  geometry_msgs::msg::Point p4;

  p1.x = center_x_ - marker_size;
  p1.y = center_y_;
  p1.z = 0.05;
  p2.x = center_x_ + marker_size;
  p2.y = center_y_;
  p2.z = 0.05;
  p3.x = center_x_;
  p3.y = center_y_ - marker_size;
  p3.z = 0.05;
  p4.x = center_x_;
  p4.y = center_y_ + marker_size;
  p4.z = 0.05;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p4);

  visualization_msgs::msg::Marker label;
  label.header = marker.header;
  label.ns = marker_namespace + "_label";
  label.id = marker_id + 10000;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position.x = center_x_;
  label.pose.position.y = center_y_;
  label.pose.position.z = 0.45;
  label.pose.orientation.w = 1.0;
  label.scale.z = 0.25;
  label.color.a = 1.0;
  label.color.r = 1.0;
  label.color.g = 1.0;
  label.color.b = 1.0;
  label.text = marker_namespace;

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_array.markers.push_back(label);
  marker_pub_->publish(marker_array);
}

BT::NodeStatus CalibrateCenterAnchorAction::tick()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }

  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  std::string point_topic;
  std::string marker_topic = "calibrated_points";
  std::string marker_namespace = "calibration_point";
  std::string marker_color = "red";
  bool manual_only = false;
  bool force_recalibrate = false;
  int marker_id = 0;
  double marker_size = 0.35;
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);
  getInput("manual_only", manual_only);
  getInput("point_topic", point_topic);
  getInput("force_recalibrate", force_recalibrate);
  getInput("marker_topic", marker_topic);
  getInput("marker_namespace", marker_namespace);
  getInput("marker_color", marker_color);
  getInput("marker_id", marker_id);
  getInput("marker_size", marker_size);

  if (!marker_pub_ || marker_topic_ != marker_topic) {
    marker_topic_ = marker_topic;
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

  updatePointSubscription(point_topic);

  if (calibrated_ && !force_recalibrate) {
    publishOutputs(map_frame);
    publishCrossMarker(map_frame, marker_namespace, marker_color, marker_id, marker_size);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "[CALIB_POINT] ns=%s color=%s center=(%.3f, %.3f) yaw=%.3f topic=%s",
      marker_namespace.c_str(), marker_color.c_str(), center_x_, center_y_, center_yaw_,
      point_topic.empty() ? "<none>" : point_topic.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (!point_topic.empty()) {
    std::optional<geometry_msgs::msg::PointStamped> clicked_point;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      if (latest_point_) {
        clicked_point = latest_point_;
      }
    }

    if (clicked_point) {
      center_x_ = clicked_point->point.x;
      center_y_ = clicked_point->point.y;
      center_yaw_ = 0.0;
      calibrated_ = true;
      publishOutputs(map_frame);
      publishCrossMarker(map_frame, marker_namespace, marker_color, marker_id, marker_size);
      RCLCPP_WARN(
        node_->get_logger(),
        "[MANUAL_CALIBRATED] ns=%s color=%s topic=%s point=(%.3f, %.3f)",
        marker_namespace.c_str(), marker_color.c_str(), point_topic.c_str(), center_x_, center_y_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (manual_only) {
    setOutput("center_valid", false);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 30000,
      "CalibrateCenterAnchor waiting manual point on topic: %s",
      point_topic.empty() ? "<empty>" : point_topic.c_str());
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);

    center_x_ = transform.transform.translation.x;
    center_y_ = transform.transform.translation.y;

    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, center_yaw_);

    calibrated_ = true;
    publishOutputs(map_frame);
    publishCrossMarker(map_frame, marker_namespace, marker_color, marker_id, marker_size);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[CENTER_CALIBRATED] ns=%s color=%s map_frame=%s base_frame=%s center=(%.3f, %.3f) yaw=%.3f rad",
      marker_namespace.c_str(), marker_color.c_str(), map_frame.c_str(), base_frame.c_str(), center_x_,
      center_y_, center_yaw_);

    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    setOutput("center_valid", false);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "CalibrateCenterAnchor waiting tf %s -> %s: %s",
      map_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::CalibrateCenterAnchorAction, "CalibrateCenterAnchor");
