#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_game_time.hpp"

namespace rm_behavior_tree
{

RmucIsGameTimeCondition::RmucIsGameTimeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&RmucIsGameTimeCondition::checkGameTime, this), config)
{
}

BT::NodeStatus RmucIsGameTimeCondition::checkGameTime()
{
  int game_progress = 0, lower_remain_time = 0, higher_remain_time = 0;
  auto msg = getInput<rm_decision_interfaces::msg::RMUCGameStatus>("message");
  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }
  if (msg->game_progress == game_progress &&
      msg->stage_remain_time >= lower_remain_time &&
      msg->stage_remain_time <= higher_remain_time) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsGameTimeCondition>("RmucIsGameTime");
}
