#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_

#include "behaviortree_cpp/contrib/json.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_behavior_tree/bt_conversions.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <string>

// PoseStamped -> JSON for Groot2 (defined inline to avoid ODR)

namespace rm_behavior_tree
{

// Allows PoseStamped to be visualized in Groot2
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


class SendGoalAction : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  SendGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "full goal pose (preferred)"),
      // Alternatively pass coordinates directly in XML: goal_x and goal_y
      BT::InputPort<double>("goal_x", 0.0, "goal x coordinate"),
      BT::InputPort<double>("goal_y", 0.0, "goal y coordinate"),
      // Frame of goal_x/goal_y (and fallback for goal_pose if header.frame_id is empty)
      BT::InputPort<std::string>("frame_id", "map", "frame_id for the goal (e.g. map/odom/chassis)"),
      // Compatibility: accept action_name from XML even though this node publishes to a topic.
      BT::InputPort<std::string>("action_name", "navigate_to_pose"),
      // Optional: throttle interval in milliseconds. Default 0 keeps old behavior (publish every tick).
      BT::InputPort<int>("min_interval_ms", 0, "minimum publish interval in ms")
    };
  }

private:
  static bool isFinite_(double v)
  {
    return std::isfinite(v);
  }

  bool isSameGoal_(const geometry_msgs::msg::PoseStamped & a,
                   const geometry_msgs::msg::PoseStamped & b) const;

  geometry_msgs::msg::PoseStamped last_goal_;
  geometry_msgs::msg::PoseStamped last_log_goal_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
  bool has_last_{false};
  bool has_log_goal_{false};
  int log_count_{0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
