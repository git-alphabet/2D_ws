#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_supply_card_detected.hpp"

namespace rm_behavior_tree
{

RmucIsSupplyCardDetectedCondition::RmucIsSupplyCardDetectedCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus RmucIsSupplyCardDetectedCondition::tick()
{
  auto res = getInput<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status");
  if (!res) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & msg = res.value();
  // RMUC.msg 使用 rfid_supply 字段（对应 RMUL.msg 的 rfid_supply_arrived）
  return msg.rfid_supply ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucIsSupplyCardDetectedCondition, "RmucIsSupplyCardDetected");
