#include "rm_behavior_tree/plugins/rmul_2026/condition/is_game_time.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
inline rclcpp::Logger bt_logger()
{
  return rclcpp::get_logger("rm_behavior_tree");
}

inline rclcpp::Clock & bt_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}
}  // namespace

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
    RCLCPP_DEBUG_THROTTLE(
      bt_logger(), bt_clock(), 3000,
      "[IsGameTime] latched=true, keep SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  int game_progress = 4;
  int lower_remain_time = 0;
  int higher_remain_time = 300;
  int initial_full_hp = 400;
  int hp_drop_delta = 1;
  bool allow_no_game_status_hp_fallback = false;

  auto msg = getInput<sp_msgs::msg::RMUL>("message");
  auto robot_msg = getInput<std::shared_ptr<sp_msgs::msg::RMUL>>("robot_msg");

  getInput("game_progress", game_progress);
  getInput("lower_remain_time", lower_remain_time);
  getInput("higher_remain_time", higher_remain_time);
  getInput("initial_full_hp", initial_full_hp);
  getInput("hp_drop_delta", hp_drop_delta);
  getInput("allow_no_game_status_hp_fallback", allow_no_game_status_hp_fallback);

  // 条件1: game_progress 正常匹配（仅在存在 game_status 时判断）
  if (msg) {
    RCLCPP_INFO_THROTTLE(
      bt_logger(), bt_clock(), 2000,
      "[IsGameTime] game_status recv: game_progress=%d stage_remain_time=%d, expect_progress=%d "
      "remain_range=[%d,%d]",
      static_cast<int>(msg->game_progress), static_cast<int>(msg->stage_remain_time), game_progress,
      lower_remain_time, higher_remain_time);

    if (
      msg->game_progress == game_progress && msg->stage_remain_time >= lower_remain_time &&
      msg->stage_remain_time <= higher_remain_time) {
      game_started_ = true;
      RCLCPP_INFO(
        bt_logger(),
        "[IsGameTime] SUCCESS by game_progress: game_progress=%d stage_remain_time=%d",
        static_cast<int>(msg->game_progress), static_cast<int>(msg->stage_remain_time));
      return BT::NodeStatus::SUCCESS;
    }
  } else if (!allow_no_game_status_hp_fallback) {
    RCLCPP_WARN_THROTTLE(
      bt_logger(), bt_clock(), 2000,
      "[IsGameTime] no game_status and hp fallback disabled -> FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // 条件2: HP 下降 → 比赛已在进行（电控丢包/无 game_status 容错）
  // 优先从 robot_msg（robot_status 话题）读取 HP。
  int cur_hp = 0;
  int is_attacked = 0;
  if (robot_msg && robot_msg.value()) {
    cur_hp = static_cast<int>(robot_msg.value()->current_hp);
    is_attacked = static_cast<int>(robot_msg.value()->is_attacked);
  } else if (msg) {
    // 回退：从 game_status 消息读（实车可能合并到同一话题）
    cur_hp = static_cast<int>(msg->current_hp);
    is_attacked = static_cast<int>(msg->is_attacked);
  } else {
    RCLCPP_WARN_THROTTLE(
      bt_logger(), bt_clock(), 2000,
      "[IsGameTime] no robot_status/game_status available for HP fallback -> FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO_THROTTLE(
    bt_logger(), bt_clock(), 2000,
    "[IsGameTime] hp_fallback probe: cur_hp=%d last_hp=%d is_attacked=%d (initial_full_hp=%d hp_drop_delta=%d)",
    cur_hp, last_hp_, is_attacked, initial_full_hp, hp_drop_delta);

  // 条件2.0: 被攻击信号触发（用于弥补掉血瞬间未采到的问题）
  if (is_attacked != 0) {
    game_started_ = true;
    RCLCPP_INFO(bt_logger(), "[IsGameTime] SUCCESS by is_attacked=%d", is_attacked);
    return BT::NodeStatus::SUCCESS;
  }

  // 条件2.1: 首次拿到血量时，若已明显低于初始满血，也视为比赛已经开始
  if (initial_full_hp > 0 && cur_hp <= (initial_full_hp - hp_drop_delta)) {
    game_started_ = true;
    RCLCPP_INFO(
      bt_logger(),
      "[IsGameTime] SUCCESS by initial HP drop: cur_hp=%d <= %d",
      cur_hp, (initial_full_hp - hp_drop_delta));
    return BT::NodeStatus::SUCCESS;
  }

  // 条件2.2: 相邻两帧血量出现下降（>= hp_drop_delta）
  if (last_hp_ >= 0 && cur_hp <= (last_hp_ - hp_drop_delta)) {
    game_started_ = true;
    RCLCPP_INFO(
      bt_logger(),
      "[IsGameTime] SUCCESS by delta HP drop: cur_hp=%d <= last_hp(%d)-delta(%d)",
      cur_hp, last_hp_, hp_drop_delta);
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
