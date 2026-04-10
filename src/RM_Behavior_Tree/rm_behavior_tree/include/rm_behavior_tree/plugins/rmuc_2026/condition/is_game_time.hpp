#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_GAME_TIME_HPP_

#include <chrono>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmuc_game_status.hpp"
#include "sp_msgs/msg/rmuc_robot_status.hpp"

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
      BT::InputPort<sp_msgs::msg::RMUCGameStatus>("message"),
      BT::InputPort<sp_msgs::msg::RMUCRobotStatus>("robot_status"),
      BT::InputPort<int>("game_progress"),
      BT::InputPort<int>("lower_remain_time"),
      BT::InputPort<int>("higher_remain_time"),
      BT::InputPort<bool>("allow_no_game_status_hp_fallback", "false")};
  }

private:
  bool game_started_{false};
  int last_hp_{-1};
  std::chrono::steady_clock::time_point last_wait_log_{};
};
}  // namespace rm_behavior_tree

#endif
