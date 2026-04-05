#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_RESPAWN_CMD_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/action_node.h"
#include "sp_msgs/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
/// 根据死亡状态、经济、比赛时间决定是否确认复活/兑换立即复活
class DecideRespawnCmdAction : public BT::SyncActionNode
{
public:
  DecideRespawnCmdAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_dead"),
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status"),
      BT::InputPort<int>("team_coins"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<int>("base_hp_cur"),
      BT::InputPort<int>("base_hp_max"),
      BT::InputPort<bool>("base_threat"),
      BT::OutputPort<int>("confirm_respawn"),
      BT::OutputPort<int>("confirm_instant_respawn")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
