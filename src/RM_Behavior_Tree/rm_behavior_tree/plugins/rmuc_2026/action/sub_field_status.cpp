#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_field_status.hpp"

namespace rm_behavior_tree
{

RmucSubFieldStatusAction::RmucSubFieldStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCFieldStatus>(name, conf, params)
{
}

BT::NodeStatus RmucSubFieldStatusAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCFieldStatus> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(logger(), "[%s] field_status received", name().c_str());
    setOutput("field_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubFieldStatusAction, "RmucSubFieldStatus");
