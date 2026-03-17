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

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
}

BT::NodeStatus CalibrateCenterAnchorAction::tick()
{
  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  std::string marker_topic = "calibrated_points";
  std::string marker_namespace = "calibration_point";
  std::string marker_color = "red";
  bool force_recalibrate = false;
  int marker_id = 0;
  double marker_size = 0.35;
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);
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

  if (calibrated_ && !force_recalibrate) {
    publishOutputs(map_frame);
    publishCrossMarker(map_frame, marker_namespace, marker_color, marker_id, marker_size);
    return BT::NodeStatus::SUCCESS;
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
      "[CENTER_CALIBRATED] map_frame=%s base_frame=%s center=(%.3f, %.3f) yaw=%.3f rad",
      map_frame.c_str(), base_frame.c_str(), center_x_, center_y_, center_yaw_);

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
