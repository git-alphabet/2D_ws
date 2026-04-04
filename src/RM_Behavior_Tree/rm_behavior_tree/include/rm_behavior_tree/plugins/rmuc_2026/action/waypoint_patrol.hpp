#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__WAYPOINT_PATROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__WAYPOINT_PATROL_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在 3 个巡逻点之间循环选择下一个最远的点作为目标
class WaypointPatrolAction : public BT::SyncActionNode
{
public:
  WaypointPatrolAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("wpt0_x"), BT::InputPort<double>("wpt0_y"),
      BT::InputPort<double>("wpt1_x"), BT::InputPort<double>("wpt1_y"),
      BT::InputPort<double>("wpt2_x"), BT::InputPort<double>("wpt2_y"),
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y")};
  }
  BT::NodeStatus tick() override;

private:
  int current_idx_{0};
};
}  // namespace rm_behavior_tree
#endif
