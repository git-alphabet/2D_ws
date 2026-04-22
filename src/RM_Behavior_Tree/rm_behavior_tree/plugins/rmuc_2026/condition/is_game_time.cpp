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
  auto robot_status = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status");

  int msg_game_progress = -1;
  int msg_remain_time = -1;
  int cur_hp = -1;

  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  getInput("allow_no_game_status_hp_fallback", allow_no_game_status_hp_fallback);

  // 条件1: game_progress 正常匹配（仅在存在 game_status 时判断）
  if (msg) {
    msg_game_progress = msg->game_progress;
    msg_remain_time = static_cast<int>(msg->stage_remain_time);
    if (msg->game_progress == game_progress &&
        msg->stage_remain_time >= lower_remain_time &&
        msg->stage_remain_time <= higher_remain_time) {
      std::cout
        << "[RmucIsGameTime] 比赛开始：由 game_status 触发"
        << " progress=" << msg->game_progress
        << " remain=" << msg->stage_remain_time
        << std::endl;
      game_started_ = true;
      return BT::NodeStatus::SUCCESS;
    }
  } else if (!allow_no_game_status_hp_fallback) {
    return BT::NodeStatus::FAILURE;
  }

  // 条件2: HP 下降 → 比赛已在进行（电控丢包/无 game_status 容错）
  if (robot_status && *robot_status) {
    cur_hp = static_cast<int>((*robot_status)->current_hp);
    if (last_hp_ >= 0 && cur_hp < last_hp_) {
      std::cout
        << "[RmucIsGameTime] 比赛开始：由 HP fallback 触发"
        << " last_hp=" << last_hp_
        << " cur_hp=" << cur_hp
        << " game_progress=" << msg_game_progress
        << " remain=" << msg_remain_time
        << std::endl;
      game_started_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    if (last_hp_ < 0) {
      std::cout
        << "[RmucIsGameTime] HP fallback 基线已记录"
        << " baseline_hp=" << cur_hp
        << std::endl;
    }

    last_hp_ = cur_hp;
  }

  // 节流日志：每 3 秒打印一次等待提示
  auto now = std::chrono::steady_clock::now();
  if (now - last_wait_log_ > std::chrono::seconds(3)) {
    std::cout
      << "[RmucIsGameTime] 比赛未开始，等待接收数据"
      << " expected_progress=" << game_progress
      << " msg_progress=" << msg_game_progress
      << " remain=" << msg_remain_time
      << " last_hp=" << last_hp_
      << " cur_hp=" << cur_hp
      << " hp_fallback=" << (allow_no_game_status_hp_fallback ? "on" : "off")
      << std::endl;
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
