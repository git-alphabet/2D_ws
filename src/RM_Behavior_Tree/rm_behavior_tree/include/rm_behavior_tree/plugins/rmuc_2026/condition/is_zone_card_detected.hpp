#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_ZONE_CARD_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_ZONE_CARD_DETECTED_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 检测 RFID 是否在指定区域（SUPPLY / BASE / OUTPOST / CONTROL 等）
class RmucIsZoneCardDetectedCondition : public BT::ConditionNode
{
public:
  RmucIsZoneCardDetectedCondition(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("zone", "SUPPLY", "区域类型"),
      // Keep legacy port names for BT XML compatibility.
      BT::InputPort<std::string>("rfid_status"),
      BT::InputPort<std::string>("robot_status")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
