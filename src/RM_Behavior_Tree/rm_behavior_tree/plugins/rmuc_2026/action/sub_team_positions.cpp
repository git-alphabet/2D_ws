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
    RCLCPP_DEBUG(
      logger(), "[%s] team_positions: hero=(%.1f,%.1f), inf3=(%.1f,%.1f)",
      name().c_str(), last_msg->hero_x, last_msg->hero_y,
      last_msg->infantry3_x, last_msg->infantry3_y);
    setOutput("team_positions", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubTeamPositionsAction, "RmucSubTeamPositions");
