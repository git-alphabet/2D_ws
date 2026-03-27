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

  int game_progress = 4;
  int lower_remain_time = 0;
  int higher_remain_time = 300;
  bool allow_no_game_status_hp_fallback = false;

  auto msg = getInput<sp_msgs::msg::RMUL>("message");
  auto robot_msg = getInput<std::shared_ptr<sp_msgs::msg::RMUL>>("robot_msg");

  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  getInput("allow_no_game_status_hp_fallback", allow_no_game_status_hp_fallback);

  // 条件1: game_progress 正常匹配（仅在存在 game_status 时判断）
  if (msg) {
    if (
      msg->game_progress == game_progress && msg->stage_remain_time >= lower_remain_time &&
      msg->stage_remain_time <= higher_remain_time) {
      game_started_ = true;
      return BT::NodeStatus::SUCCESS;
    }
  } else if (!allow_no_game_status_hp_fallback) {
    return BT::NodeStatus::FAILURE;
  }

  // 条件2: HP 下降 → 比赛已在进行（电控丢包/无 game_status 容错）
  // 优先从 robot_msg（robot_status 话题）读取 HP。
  int cur_hp = 0;
  if (robot_msg && robot_msg.value()) {
    cur_hp = static_cast<int>(robot_msg.value()->current_hp);
  } else if (msg) {
    // 回退：从 game_status 消息读（实车可能合并到同一话题）
    cur_hp = static_cast<int>(msg->current_hp);
  } else {
    return BT::NodeStatus::FAILURE;
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
