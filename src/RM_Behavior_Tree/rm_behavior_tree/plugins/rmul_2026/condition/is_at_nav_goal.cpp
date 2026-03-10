#include "rm_behavior_tree/plugins/rmul_2026/condition/is_at_nav_goal.hpp"

namespace rm_behavior_tree
{

IsAtNavGoal::IsAtNavGoal(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsAtNavGoal::tick()
{
  // ------------------------------------------------
  // 1) 从黑板读取 RMUL.msg
  // ------------------------------------------------
  rm_decision_interfaces::msg::RMUL rfid_msg;
  auto res = getInput<rm_decision_interfaces::msg::RMUL>("rfid_status");
  if (!res) {
    // 黑板没有该 key / 类型不匹配 / 尚未写入：保守失败
    return BT::NodeStatus::FAILURE;
  }
  rfid_msg = res.value();

  // ------------------------------------------------
  // 2) 检查是否到达导航目标点
  // ------------------------------------------------
  const bool is_at_goal = rfid_msg.is_at_nav_goal;

  // ------------------------------------------------
  // 3) 到达 -> SUCCESS；未到达 -> FAILURE
  // ------------------------------------------------
  return is_at_goal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAtNavGoal>("IsAtNavGoal");
}
