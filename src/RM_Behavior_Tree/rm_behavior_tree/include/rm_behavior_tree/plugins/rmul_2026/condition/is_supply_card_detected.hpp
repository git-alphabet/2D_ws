#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

class IsSupplyCardDetectedCondition : public BT::ConditionNode
{
public:
  IsSupplyCardDetectedCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUL>("rfid_status")
    };
  }

  BT::NodeStatus tick() override;

private:
  // 保留 params_ 是为了与 CreateRosNodePlugin 的构造签名完全一致
  // 如果你们开了非常严格的告警并把未使用当错误，我也可以给你“无成员变量”的版本
  BT::RosNodeParams params_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_
