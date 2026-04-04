#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_GAME_TIME_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmuc_game_status.hpp"

namespace rm_behavior_tree
{
class RmucIsGameTimeCondition : public BT::SimpleConditionNode
{
public:
  RmucIsGameTimeCondition(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus checkGameTime();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUCGameStatus>("message"),
      BT::InputPort<int>("game_progress"),
      BT::InputPort<int>("lower_remain_time"),
      BT::InputPort<int>("higher_remain_time")};
  }
};
}  // namespace rm_behavior_tree

#endif
