#include "rm_behavior_tree/plugins/rmul_2026/condition/is_game_time.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace rm_behavior_tree
{

IsGameTimeCondition::IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameTimeCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameTimeCondition::checkGameStart()
{
  // 已锁存则直接返回
  if (game_started_) {
    return BT::NodeStatus::SUCCESS;
  }

  int game_progress, lower_remain_time, higher_remain_time;
  auto msg = getInput<sp_msgs::msg::RMUL>("message");
  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 条件1: game_progress 正常匹配
  if (
    msg->game_progress == game_progress && msg->stage_remain_time >= lower_remain_time &&
    msg->stage_remain_time <= higher_remain_time) {
    game_started_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // 条件2: HP 下降 → 比赛已在进行（电控丢包容错）
  // 优先从 robot_msg（robot_status 话题）读取 HP，因为 game_status 话题不含 HP
  int cur_hp = 0;
  auto robot_msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("robot_msg");
  if (robot_msg && *robot_msg) {
    cur_hp = static_cast<int>((*robot_msg)->current_hp);
  } else {
    // 回退：从 game_status 消息读（实车可能合并到同一话题）
    cur_hp = static_cast<int>(msg->current_hp);
  }
  if (last_hp_ >= 0 && cur_hp < last_hp_) {
    game_started_ = true;
    return BT::NodeStatus::SUCCESS;
  }
  last_hp_ = cur_hp;

  return BT::NodeStatus::FAILURE;
}
}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsGameTimeCondition>("IsGameTime");
}
