#include "rm_behavior_tree/plugins/rmul_2026/condition/is_supply_card_detected.hpp"

namespace rm_behavior_tree
{

IsSupplyCardDetectedCondition::IsSupplyCardDetectedCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsSupplyCardDetectedCondition::tick()
{
  auto res = getInput<rm_decision_interfaces::msg::RMUL>("rfid_status");
  if (!res) {
    // 黑板没有该数据：保守认为没刷到补给区卡
    return BT::NodeStatus::FAILURE;
  }

  const auto & msg = res.value();
  // rm_decision_interfaces::msg::RMUL currently provides fields:
  //   bool rfid_supply_arrived
  //   bool rfid_control_arrived
  // 判断补给区刷卡应使用 `rfid_supply_arrived` 字段
  return msg.rfid_supply_arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsSupplyCardDetectedCondition, "IsSupplyCardDetected");
