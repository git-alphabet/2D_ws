#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_team_positions.hpp"

namespace rm_behavior_tree
{

RmucSubTeamPositionsAction::RmucSubTeamPositionsAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCTeamPositions>(name, conf, params)
{
}

BT::NodeStatus RmucSubTeamPositionsAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCTeamPositions> & last_msg)
{
  if (last_msg) {
    setOutput("team_positions", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubTeamPositionsAction, "RmucSubTeamPositions");
