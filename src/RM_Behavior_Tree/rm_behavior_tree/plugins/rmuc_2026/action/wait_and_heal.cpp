#include "rm_behavior_tree/plugins/rmuc_2026/action/wait_and_heal.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

RmucWaitAndHealAction::RmucWaitAndHealAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotStatus>(name, conf, params)
{
}

BT::NodeStatus RmucWaitAndHealAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> & last_msg)
{
  // 1) 更新血量缓存
  if (last_msg) {
    int hp = static_cast<int>(last_msg->current_hp);
    hp = std::clamp(hp, 0, MAX_HP_FIXED);
    last_hp_cache_ = hp;
    has_hp_cache_ = true;

    RCLCPP_DEBUG(
      logger(), "[%s] new RMUC msg, current_hp=%d", name().c_str(), last_hp_cache_);
  }

  // 2) 若还没有任何血量数据 -> 继续等待
  if (!has_hp_cache_) {
    return BT::NodeStatus::RUNNING;
  }

  // 3) 读取参数
  std::uint64_t heal_wait_ms = 0;
  if (auto res = getInput<std::uint64_t>("heal_wait_ms")) {
    heal_wait_ms = res.value();
  }

  double heal_min_ratio = 0.00;
  if (auto res = getInput<double>("heal_min_ratio")) {
    heal_min_ratio = res.value();
  }
  heal_min_ratio = std::clamp(heal_min_ratio, 0.0, 1.0);

  // 4) 当前时间（毫秒）
  const std::uint64_t now_ms =
    static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  // 5) 当前血量
  const int current_hp = last_hp_cache_;

  if (current_hp <= 0) {
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
    return BT::NodeStatus::FAILURE;
  }

  // 6) 读取/初始化 heal_start_ms
  std::uint64_t heal_start_ms = 0;
  if (auto res = getInput<std::uint64_t>("heal_start_ms")) {
    heal_start_ms = res.value();
  }

  if (heal_start_ms == 0ULL) {
    heal_start_ms = now_ms;
    setOutput("heal_start_ms", heal_start_ms);
  }

  // 7) elapsed 保护
  const std::uint64_t elapsed_ms =
    (now_ms >= heal_start_ms) ? (now_ms - heal_start_ms) : 0ULL;

  // 8) 血量比例
  const double hp_ratio =
    static_cast<double>(current_hp) / static_cast<double>(MAX_HP_FIXED);

  // 9) 判定
  const bool time_ok = (elapsed_ms >= heal_wait_ms);
  const bool hp_ok = (hp_ratio >= heal_min_ratio);

  if (time_ok && hp_ok) {
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucWaitAndHealAction, "RmucWaitAndHeal");
