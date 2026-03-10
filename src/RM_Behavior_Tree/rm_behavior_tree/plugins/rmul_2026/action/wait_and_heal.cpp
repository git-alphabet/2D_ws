#include "rm_behavior_tree/plugins/rmul_2026/action/wait_and_heal.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

WaitAndHealAction::WaitAndHealAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUL>(name, conf, params)
{
}

BT::NodeStatus WaitAndHealAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUL> & last_msg)
{
  // 1) 更新血量缓存（仅当有新消息）
  if (last_msg) {
    int hp = static_cast<int>(last_msg->current_hp);
    // clamp函数确保血量在合理范围内，小于0视为0，大于最大值视为最大值，范围之内取原值
    hp = std::clamp(hp, 0, MAX_HP_FIXED);
    last_hp_cache_ = hp;
    has_hp_cache_ = true;

    RCLCPP_DEBUG(
      logger(), "[%s] new RMUL msg, current_hp=%d", name().c_str(), last_hp_cache_);
  }

  // 2) 若还没有任何血量数据，无法判断 -> 继续等待
  if (!has_hp_cache_) {
    return BT::NodeStatus::RUNNING;
  }

  // 3) 读取参数：缺失时给保守默认值（比赛更稳）
  std::uint64_t heal_wait_ms = 0;// 用来设置等待回血的时间，默认2秒
  if (auto res = getInput<std::uint64_t>("heal_wait_ms")) {
    heal_wait_ms = res.value();
  }

  double heal_min_ratio = 0.00;// 用来设置最低回血比例，默认60%
  if (auto res = getInput<double>("heal_min_ratio")) {
    heal_min_ratio = res.value();
  }
  heal_min_ratio = std::clamp(heal_min_ratio, 0.0, 1.0);

  // 4) ✅ now_ms 直接从 ROS 获取（毫秒）
  const std::uint64_t now_ms =
    static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  // 5) 当前血量（来自缓存）
  const int current_hp = last_hp_cache_;

  // 如果又死了：清理回血计时并失败（上层会切回战亡逻辑）
  if (current_hp <= 0) {
    setOutput("heal_start_ms", 0ULL);
    return BT::NodeStatus::FAILURE;
  }

  // 6) 读取/初始化 heal_start_ms（inout）
  std::uint64_t heal_start_ms = 0;// 记录开始等待回血的时间戳
  if (auto res = getInput<std::uint64_t>("heal_start_ms")) {
    heal_start_ms = res.value();
  }

  if (heal_start_ms == 0ULL) {
    heal_start_ms = now_ms;
    setOutput("heal_start_ms", heal_start_ms);
  }

  // 7) elapsed 保护：防止时间回跳导致负数
  const std::uint64_t elapsed_ms =
    (now_ms >= heal_start_ms) ? (now_ms - heal_start_ms) : 0ULL;

  // 8) 血量比例：hp / 400
  const double hp_ratio =
    static_cast<double>(current_hp) / static_cast<double>(MAX_HP_FIXED);

  // 9) 判定：时间门 + 血量门都满足才结束（更稳）
  const bool time_ok = (elapsed_ms >= heal_wait_ms);
  const bool hp_ok = (hp_ratio >= heal_min_ratio);

  if (time_ok && hp_ok) {
    // 成功结束：清零计时，方便下次复活流程复用
    setOutput("heal_start_ms", 0ULL);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::WaitAndHealAction, "WaitAndHeal");
