#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_CMD_STATE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_CMD_STATE_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 初始化指令状态黑板变量 (cmd_state, allow_ammo_target)
class InitCmdStateAction : public BT::SyncActionNode
{
public:
  InitCmdStateAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::string>("cmd_state", std::string(""), "cmd state"),
      BT::BidirectionalPort<int>("allow_ammo_target", 0, "allow ammo target")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
