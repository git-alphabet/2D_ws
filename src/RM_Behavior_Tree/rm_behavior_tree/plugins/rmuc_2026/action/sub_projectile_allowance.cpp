#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_projectile_allowance.hpp"

namespace rm_behavior_tree
{

RmucSubProjectileAllowanceAction::RmucSubProjectileAllowanceAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCProjectileAllowance>(name, conf, params)
{
}

BT::NodeStatus RmucSubProjectileAllowanceAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCProjectileAllowance> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] projectile_allowance: fortress_ammo=%d",
      name().c_str(), last_msg->fortress_ammo);
    setOutput("projectile_allowance", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(
  rm_behavior_tree::RmucSubProjectileAllowanceAction, "RmucSubProjectileAllowance");
