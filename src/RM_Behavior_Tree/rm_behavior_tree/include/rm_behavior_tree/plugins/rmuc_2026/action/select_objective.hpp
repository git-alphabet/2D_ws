#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 根据比赛局势选择下一个战略目标点（中心高地/梯形高地/敌方堡垒等）
class SelectObjectiveAction : public BT::SyncActionNode
{
public:
  SelectObjectiveAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<int>("hp_cur"), BT::InputPort<int>("hp_max"),
      BT::InputPort<bool>("outpost_alive"),
      BT::InputPort<bool>("enemy_outpost_destroyed"),
      BT::InputPort<double>("cap_outpost_x"),
      BT::InputPort<double>("cap_outpost_y"),
      BT::InputPort<double>("central_highland_x"),
      BT::InputPort<double>("central_highland_y"),
      BT::InputPort<double>("ladder_highland_x"),
      BT::InputPort<double>("ladder_highland_y"),
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y"),
      BT::OutputPort<std::string>("objective_name")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
