#include "rm_behavior_tree/plugins/rmul_2026/condition/navigate_to_goal.hpp"

namespace rm_behavior_tree
{

NavigateToGoal::NavigateToGoal(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  // 纯判断节点：不依赖 ROS node / logger 等资源
}

BT::NodeStatus NavigateToGoal::tick()
{
  // ------------------------------------------------
  // 1) 从黑板读取 RFID.msg
  // ------------------------------------------------
  rm_decision_interfaces::msg::RMUL rfid_msg;
  if (!getInput<rm_decision_interfaces::msg::RMUL>("rfid_status", rfid_msg)) {
    // 黑板上没有该 key / 类型不匹配 / 尚未写入消息：保守失败
    return BT::NodeStatus::FAILURE;
  }

  // ------------------------------------------------
  // 2) 仅判断一个布尔量：true -> SUCCESS, false -> FAILURE
  // ------------------------------------------------
  return rfid_msg.rfid_supply_arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::NavigateToGoal>("NavigateToGoal");
}
