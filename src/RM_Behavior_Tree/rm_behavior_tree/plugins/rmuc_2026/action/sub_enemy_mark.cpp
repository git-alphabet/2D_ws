#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_enemy_mark.hpp"

namespace rm_behavior_tree
{

RmucSubEnemyMarkAction::RmucSubEnemyMarkAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCEnemyMark>(name, conf, params)
{
}

BT::NodeStatus RmucSubEnemyMarkAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCEnemyMark> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(logger(), "[%s] enemy_mark received", name().c_str());
    setOutput("enemy_mark", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubEnemyMarkAction, "RmucSubEnemyMark");
