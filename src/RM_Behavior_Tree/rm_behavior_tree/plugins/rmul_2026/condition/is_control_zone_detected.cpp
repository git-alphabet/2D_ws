#include "rm_behavior_tree/plugins/rmul_2026/condition/is_control_zone_detected.hpp"

namespace rm_behavior_tree
{

IsControlZoneDetectedCondition::IsControlZoneDetectedCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsControlZoneDetectedCondition::tick()
{
  auto res = getInput<rm_decision_interfaces::msg::RMUL>("rfid_status");
  if (!res) {
    // 黑板没有该数据：保守认为没有与控制区产生交互
    return BT::NodeStatus::FAILURE;
  }

  const auto & msg = res.value();
  // rm_decision_interfaces::msg::RMUL provides:
  //   bool rfid_supply_arrived   - 补给区交互卡反馈
  //   bool rfid_control_arrived  - 控制区交互卡反馈
  // 判断控制区交互应使用 `rfid_control_arrived` 字段
  return msg.rfid_control_arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsControlZoneDetectedCondition, "IsControlZoneDetected");
