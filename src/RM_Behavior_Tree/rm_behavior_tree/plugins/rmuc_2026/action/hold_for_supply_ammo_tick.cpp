#include "rm_behavior_tree/plugins/rmuc_2026/action/hold_for_supply_ammo_tick.hpp"

namespace rm_behavior_tree
{

HoldForSupplyAmmoTickAction::HoldForSupplyAmmoTickAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus HoldForSupplyAmmoTickAction::onStart()
{
  initial_ammo_ = 0;
  getInput("ammo_allow", initial_ammo_);
  start_time_ = std::chrono::steady_clock::now();
  return onRunning();
}

BT::NodeStatus HoldForSupplyAmmoTickAction::onRunning()
{
  int ammo = 0;
  getInput("ammo_allow", ammo);

  // 弹药增加了 → 领取成功，重置重试状态
  if (ammo > initial_ammo_) {
    int default_thr = 80;
    getInput("default_threshold", default_thr);
    setOutput("supply_fail_count", 0);
    setOutput("supply_next_threshold", default_thr);
    return BT::NodeStatus::SUCCESS;
  }

  // 检查超时
  int timeout_s = 30;
  getInput("timeout_s", timeout_s);
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed >= std::chrono::seconds(timeout_s)) {
    // 超时：降级阈值
    int fail_count = 0;
    getInput("supply_fail_count", fail_count);
    ++fail_count;
    setOutput("supply_fail_count", fail_count);

    if (fail_count == 1) {
      // 第1次失败：阈值降为当前弹药的一半（至少1，确保ammo=0时能触发）
      setOutput("supply_next_threshold", std::max(initial_ammo_ / 2, 1));
    } else if (fail_count == 2) {
      // 第2次失败：阈值降为1（弹药打到0时触发）
      setOutput("supply_next_threshold", 1);
    } else {
      // 第3次失败：彻底放弃（-1永远不触发）
      setOutput("supply_next_threshold", -1);
    }
    return BT::NodeStatus::SUCCESS;  // 退出补弹，回去打
  }

  return BT::NodeStatus::RUNNING;
}

void HoldForSupplyAmmoTickAction::onHalted() {}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::HoldForSupplyAmmoTickAction>("HoldForSupplyAmmoTick");
}
