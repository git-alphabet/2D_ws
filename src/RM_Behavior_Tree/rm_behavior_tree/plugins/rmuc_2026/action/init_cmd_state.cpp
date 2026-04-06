#include "rm_behavior_tree/plugins/rmuc_2026/action/init_cmd_state.hpp"

namespace rm_behavior_tree
{

InitCmdStateAction::InitCmdStateAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus InitCmdStateAction::tick()
{
  // 初始化指令状态为空字符串和 0（仅首次设置）
  std::string state;
  if (!getInput("cmd_state", state) || state.empty()) {
    setOutput("cmd_state", std::string("IDLE"));
  }
  int ammo_target = 0;
  if (!getInput("allow_ammo_target", ammo_target)) {
    setOutput("allow_ammo_target", 0);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::InitCmdStateAction>("InitCmdState");
}
