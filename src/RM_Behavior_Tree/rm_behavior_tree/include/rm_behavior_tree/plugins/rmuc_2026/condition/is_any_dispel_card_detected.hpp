#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_ANY_DISPEL_CARD_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_ANY_DISPEL_CARD_DETECTED_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmucrfid_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
/// 检测是否踩到任意可祛弱的 RFID 卡（供应/基地增益/前哨增益）
class IsAnyDispelCardDetectedCondition : public BT::ConditionNode
{
public:
  IsAnyDispelCardDetectedCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status"),
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("robot_status")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
