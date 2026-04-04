#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_BEST_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_BEST_TARGET_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmuc_enemy_tracks.hpp"

namespace rm_behavior_tree
{
/// 从雷达跟踪数据中选择最佳攻击目标（优先基地附近敌人，否则最近）
class SelectBestTargetAction : public BT::SyncActionNode
{
public:
  SelectBestTargetAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("base_x"),
      BT::InputPort<double>("base_y"),
      BT::OutputPort<std::string>("out_target")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
