#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_FROM_CENTER_OFFSET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_FROM_CENTER_OFFSET_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{

class SetGoalFromCenterOffsetAction : public BT::SyncActionNode
{
public:
  SetGoalFromCenterOffsetAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("center_x"),
      BT::InputPort<double>("center_y"),
      BT::InputPort<double>("center_yaw", 0.0, "center yaw in rad"),
      BT::InputPort<double>("offset_x"),
      BT::InputPort<double>("offset_y"),
      BT::InputPort<bool>("rotate_by_center_yaw", true, "rotate offset by center yaw"),
      BT::InputPort<std::string>("frame_id", "map", "goal frame"),
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_FROM_CENTER_OFFSET_HPP_
