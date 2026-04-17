#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AT_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AT_GOAL_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 判断是否到达导航目标（欧氏距离 < arrive_radius）
class IsAtGoalCondition : public BT::ConditionNode
{
public:
  IsAtGoalCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("goal_x"),
      BT::InputPort<double>("goal_y"),
      BT::InputPort<double>("arrive_radius", "0.35", "arrive_radius")};
  }
  BT::NodeStatus tick() override;

private:
  bool last_at_goal_{false};
  bool first_print_{true};
};
}  // namespace rm_behavior_tree
#endif
