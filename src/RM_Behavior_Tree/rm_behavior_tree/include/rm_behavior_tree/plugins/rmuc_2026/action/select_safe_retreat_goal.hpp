#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_SAFE_RETREAT_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_SAFE_RETREAT_GOAL_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在补给区和防御锚点中选择距离自身最近的安全撤退点
class SelectSafeRetreatGoalAction : public BT::SyncActionNode
{
public:
  SelectSafeRetreatGoalAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("supply_x"),
      BT::InputPort<double>("supply_y"),
      BT::InputPort<double>("defend_anchor_x"),
      BT::InputPort<double>("defend_anchor_y"),
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
