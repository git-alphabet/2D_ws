#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_

#include "behaviortree_cpp/contrib/json.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <string>

namespace rm_behavior_tree
{

inline void PoseStampedToJson(nlohmann::json & j, const geometry_msgs::msg::PoseStamped & p)
{
  j["position_x"] = p.pose.position.x;
  j["position_y"] = p.pose.position.y;
  j["position_z"] = p.pose.position.z;
  j["orientation_x"] = p.pose.orientation.x;
  j["orientation_y"] = p.pose.orientation.y;
  j["orientation_z"] = p.pose.orientation.z;
  j["orientation_w"] = p.pose.orientation.w;
}

class SendGoalAction : public BT::SyncActionNode
{
public:
  SendGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "goal_pose", "Topic name to publish goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "full goal pose (preferred)"),
      BT::InputPort<double>("goal_x", 0.0, "goal x coordinate"),
      BT::InputPort<double>("goal_y", 0.0, "goal y coordinate"),
      BT::InputPort<std::string>("frame_id", "map", "frame_id for the goal (e.g. map/odom/chassis)"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose"),
      BT::InputPort<int>("min_interval_ms", 0, "minimum publish interval in ms")
    };
  }
  
  BT::NodeStatus tick() override;

private:
  static bool isFinite_(double v)
  {
    return std::isfinite(v);
  }

  bool isSameGoal_(const geometry_msgs::msg::PoseStamped & a,
                   const geometry_msgs::msg::PoseStamped & b) const;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string prev_topic_name_;

  geometry_msgs::msg::PoseStamped last_goal_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
  bool has_last_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
