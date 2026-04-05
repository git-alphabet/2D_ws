#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_any_dispel_card_detected.hpp"

namespace rm_behavior_tree
{

IsAnyDispelCardDetectedCondition::IsAnyDispelCardDetectedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsAnyDispelCardDetectedCondition::tick()
{
  auto rfid = getInput<sp_msgs::msg::RMUCRFIDStatus>("rfid_status");
  if (!rfid) return BT::NodeStatus::FAILURE;

  // 任意可祛弱的区域 RFID 为 true → SUCCESS
  if (rfid->rfid_supply || rfid->rfid_base_buff || rfid->rfid_outpost_buff) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAnyDispelCardDetectedCondition>("IsAnyDispelCardDetected");
}
