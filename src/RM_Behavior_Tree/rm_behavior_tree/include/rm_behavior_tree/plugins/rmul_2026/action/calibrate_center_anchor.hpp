#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALIBRATE_CENTER_ANCHOR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALIBRATE_CENTER_ANCHOR_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rm_behavior_tree
{

class CalibrateCenterAnchorAction : public BT::SyncActionNode
{
public:
  CalibrateCenterAnchorAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("map_frame", "map", "map frame"),
      BT::InputPort<std::string>("base_frame", "base_footprint", "base frame"),
      BT::InputPort<bool>("force_recalibrate", false, "force recalibration each tick"),
      BT::InputPort<std::string>("marker_topic", "calibrated_points", "marker array topic"),
      BT::InputPort<std::string>("marker_namespace", "calibration_point", "marker namespace"),
      BT::InputPort<std::string>("marker_color", "red", "cross color: red or black"),
      BT::InputPort<int>("marker_id", 0, "marker id"),
      BT::InputPort<double>("marker_size", 0.35, "cross half size"),
      BT::OutputPort<double>("center_x"),
      BT::OutputPort<double>("center_y"),
      BT::OutputPort<double>("center_yaw"),
      BT::OutputPort<bool>("center_valid"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("center_pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string marker_topic_{"calibrated_points"};

  bool calibrated_{false};
  double center_x_{0.0};
  double center_y_{0.0};
  double center_yaw_{0.0};

  void publishOutputs(const std::string & map_frame);
  void publishCrossMarker(
    const std::string & map_frame,
    const std::string & marker_namespace,
    const std::string & marker_color,
    int marker_id,
    double marker_size);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALIBRATE_CENTER_ANCHOR_HPP_
