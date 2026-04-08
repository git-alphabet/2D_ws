#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_TEAM_POSITIONS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_TEAM_POSITIONS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sp_msgs/msg/rmuc_team_positions.hpp"

namespace rm_behavior_tree
{
class RmucSubTeamPositionsAction
  : public BT::RosTopicSubNode<sp_msgs::msg::RMUCTeamPositions>
{
public:
  RmucSubTeamPositionsAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<sp_msgs::msg::RMUCTeamPositions>("team_positions")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<sp_msgs::msg::RMUCTeamPositions> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
