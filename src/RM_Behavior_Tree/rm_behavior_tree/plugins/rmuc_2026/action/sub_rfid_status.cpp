#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_rfid_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

RmucSubRFIDStatusAction::RmucSubRFIDStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCRFIDStatus>(name, conf, params)
{
}

BT::NodeStatus RmucSubRFIDStatusAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCRFIDStatus> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] new RMUC msg, rfid_supply: %s", name().c_str(),
      (last_msg->rfid_supply ? "true" : "false"));
    setOutput("rfid_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubRFIDStatusAction, "RmucSubRFIDStatus");
