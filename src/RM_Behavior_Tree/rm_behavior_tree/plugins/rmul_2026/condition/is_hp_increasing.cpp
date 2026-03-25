#include "rm_behavior_tree/plugins/rmul_2026/condition/is_hp_increasing.hpp"

namespace rm_behavior_tree
{

BT::NodeStatus IsHPIncreasingCondition::tick()
{
  auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message");
  if (!msg || !(*msg)) {
    return BT::NodeStatus::FAILURE;
  }

  const int current_hp = static_cast<int>((*msg)->current_hp);

  if (last_hp_ >= 0 && current_hp > last_hp_) {
    ++increase_count_;
  } else {
    increase_count_ = 0;
  }
  last_hp_ = current_hp;

  // 连续2次HP增加才确认（避免抖动误判）
  return (increase_count_ >= 2) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsHPIncreasingCondition>("IsHPIncreasing");
}
