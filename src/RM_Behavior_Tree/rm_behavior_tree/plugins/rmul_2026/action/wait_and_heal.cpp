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
  if (last_msg) {
    int hp = static_cast<int>(last_msg->current_hp);
    hp = std::clamp(hp, 0, MAX_HP_FIXED);
    last_hp_cache_ = hp;
    has_hp_cache_ = true;

    RCLCPP_DEBUG(
      logger(), "[%s] new RMUL msg, current_hp=%d", name().c_str(), last_hp_cache_);
  }

  if (!has_hp_cache_) {
    return BT::NodeStatus::FAILURE;
  }

  std::uint64_t heal_wait_ms = 0;
  if (auto res = getInput<std::uint64_t>("heal_wait_ms")) {
    heal_wait_ms = res.value();
  }

  int heal_min_hp = 400;
  if (auto res = getInput<int>("heal_min_hp")) {
    heal_min_hp = res.value();
  }
  heal_min_hp = std::clamp(heal_min_hp, 0, MAX_HP_FIXED);

  const std::uint64_t now_ms =
    static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  const int current_hp = last_hp_cache_;

  if (current_hp <= 0) {
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
    return BT::NodeStatus::FAILURE;
  }

  std::uint64_t heal_start_ms = 0;
  if (auto res = getInput<std::uint64_t>("heal_start_ms")) {
    heal_start_ms = res.value();
  }

  if (heal_start_ms == 0ULL) {
    heal_start_ms = now_ms;
    setOutput("heal_start_ms", heal_start_ms);
  }

  const std::uint64_t elapsed_ms =
    (now_ms >= heal_start_ms) ? (now_ms - heal_start_ms) : 0ULL;

  const bool time_ok = (elapsed_ms >= heal_wait_ms);
  const bool hp_ok = (current_hp >= heal_min_hp);

  if (time_ok && hp_ok) {
    // 成功结束：清零计时，方便下次复活流程复用
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::WaitAndHealAction, "WaitAndHeal");
