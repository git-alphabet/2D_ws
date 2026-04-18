#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_zone_card_detected.hpp"

namespace rm_behavior_tree
{

RmucIsZoneCardDetectedCondition::RmucIsZoneCardDetectedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus RmucIsZoneCardDetectedCondition::tick()
{
  // All RFID fields removed from RMUCRFIDStatus.msg — always FAILURE
  (void)getInput<std::string>("zone");
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsZoneCardDetectedCondition>("IsZoneCardDetected");
}
