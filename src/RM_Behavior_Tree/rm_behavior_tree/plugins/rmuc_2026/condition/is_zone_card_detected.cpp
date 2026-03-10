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
  std::string zone = "SUPPLY";
  getInput("zone", zone);

  auto rfid = getInput<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status");
  if (!rfid) {
    return BT::NodeStatus::FAILURE;
  }

  bool detected = false;
  if (zone == "SUPPLY") {
    detected = rfid->rfid_supply;
  } else if (zone == "BASE") {
    detected = rfid->rfid_base_buff;
  } else if (zone == "OUTPOST") {
    detected = rfid->rfid_outpost_buff;
  } else if (zone == "CENTRAL_HIGHLAND") {
    detected = rfid->rfid_central_highland;
  } else if (zone == "TRAPEZOIDAL_HIGHLAND") {
    detected = rfid->rfid_ladder_highland;
  } else if (zone == "ENEMY_FORTRESS") {
    detected = rfid->rfid_fortress_enemy;
  }

  return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsZoneCardDetectedCondition>("IsZoneCardDetected");
}
