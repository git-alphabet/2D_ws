#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_game_time.hpp"
#include <iostream>

namespace rm_behavior_tree
{

RmucIsGameTimeCondition::RmucIsGameTimeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&RmucIsGameTimeCondition::checkGameTime, this), config)
{
}

BT::NodeStatus RmucIsGameTimeCondition::checkGameTime()
{
  // 已锁存则直接返回
  if (game_started_) {
    return BT::NodeStatus::SUCCESS;
  }

  int game_progress = 0, lower_remain_time = 0, higher_remain_time = 0;
  bool allow_no_game_status_hp_fallback = false;

  auto msg = getInput<sp_msgs::msg::RMUCGameStatus>("message");
  auto robot_status = getInput<sp_msgs::msg::RMUCRobotStatus>("robot_status");

  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  getInput("allow_no_game_status_hp_fallback", allow_no_game_status_hp_fallback);

  // 条件1: game_progress 正常匹配（仅在存在 game_status 时判断）
  if (msg) {
    if (msg->game_progress == game_progress &&
        msg->stage_remain_time >= lower_remain_time &&
        msg->stage_remain_time <= higher_remain_time) {
      game_started_ = true;
      return BT::NodeStatus::SUCCESS;
    }
  } else if (!allow_no_game_status_hp_fallback) {
    return BT::NodeStatus::FAILURE;
  }

  // 条件2: HP 下降 → 比赛已在进行（电控丢包/无 game_status 容错）
  if (robot_status) {
    int cur_hp = static_cast<int>(robot_status->current_hp);
    if (last_hp_ >= 0 && cur_hp < last_hp_) {
      game_started_ = true;
      return BT::NodeStatus::SUCCESS;
    }
    last_hp_ = cur_hp;
  }

  // 节流日志：每 3 秒打印一次等待提示
  auto now = std::chrono::steady_clock::now();
  if (now - last_wait_log_ > std::chrono::seconds(3)) {
    std::cout << "[RmucIsGameTime] 比赛未开始，等待接收数据" << std::endl;
    last_wait_log_ = now;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsGameTimeCondition>("RmucIsGameTime");
}
