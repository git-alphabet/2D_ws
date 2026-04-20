#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 根据死亡状态、基地血量、经济、比赛时间决定是否确认复活/兑换立即复活
class DecideRespawnCmdAction : public BT::SyncActionNode
{
public:
  DecideRespawnCmdAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_dead"),
      BT::OutputPort<int>("confirm_respawn")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
