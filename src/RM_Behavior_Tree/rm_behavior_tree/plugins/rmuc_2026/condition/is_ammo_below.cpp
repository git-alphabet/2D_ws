#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_ammo_below.hpp"
#include "behaviortree_cpp/blackboard.h"
#include <chrono>
#include <cstdio>

namespace rm_behavior_tree
{

IsAmmoBelowCondition::IsAmmoBelowCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsAmmoBelowCondition::tick()
{
  int ammo = 300, low = 80;
  getInput("ammo_allow", ammo);

  // 从根黑板直接读取动态阈值（绕过 SubTree autoremap 链）
  auto* root_bb = config().blackboard->rootBlackboard();
  try {
    low = root_bb->get<int>("supply.next_threshold");
  } catch (...) {
    getInput("ammo_low", low);
  }

  bool below = (ammo < low);

  // Throttle log: print at most once every 5 seconds
  static auto last_log = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
    std::printf("[IsAmmoBelow] ammo=%d threshold=%d → %s\n",
                ammo, low, below ? "BELOW(补弹)" : "OK");
    last_log = now;
  }

  return below ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAmmoBelowCondition>("IsAmmoBelow");
}
