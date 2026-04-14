#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_POSITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_POSITION_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sp_msgs/msg/rmuc_robot_position.hpp"

namespace rm_behavior_tree
{
class RmucSubRobotPositionAction : public BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotPosition>
{
public:
  RmucSubRobotPositionAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "robot_position", "订阅的话题名"),
      BT::OutputPort<double>("pose_x"),
      BT::OutputPort<double>("pose_y"),
      BT::OutputPort<double>("pose_yaw"),
      BT::OutputPort<bool>("is_at_nav_goal")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<sp_msgs::msg::RMUCRobotPosition> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
