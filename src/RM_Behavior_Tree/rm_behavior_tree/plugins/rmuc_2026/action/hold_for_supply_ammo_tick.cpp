#include "rm_behavior_tree/plugins/rmuc_2026/action/hold_for_supply_ammo_tick.hpp"
#include "behaviortree_cpp/blackboard.h"
#include <cstdio>

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
  last_log_time_ = start_time_;

  int timeout_s = 30;
  getInput("timeout_s", timeout_s);
  std::printf("[HoldForSupplyAmmoTick] 进入补弹倒计时 | 初始弹药=%d 超时=%ds\n", initial_ammo_, timeout_s);

  return onRunning();
}

BT::NodeStatus HoldForSupplyAmmoTickAction::onRunning()
{
  int ammo = 0;
  getInput("ammo_allow", ammo);

  // 直接操作根黑板，绕过 SubTree autoremap 链
  auto* root_bb = config().blackboard->rootBlackboard();

  // 确保根黑板上 key 有值（autoremap 可能创建了空条目，getEntry 非空但无值）
  auto ensure_init = [&](const char* key, int default_val) {
    try { root_bb->get<int>(key); }
    catch (...) { root_bb->set<int>(key, default_val); }
  };
  ensure_init("supply.fail_count", 0);
  {
    int default_thr = 80;
    getInput("default_threshold", default_thr);
    ensure_init("supply.next_threshold", default_thr);
  }

  // 弹药增加了 → 领取成功，重置重试状态
  if (ammo > initial_ammo_) {
    int default_thr = 80;
    getInput("default_threshold", default_thr);
    root_bb->set<int>("supply.fail_count", 0);
    root_bb->set<int>("supply.next_threshold", default_thr);
    std::printf("[HoldForSupplyAmmoTick] 补弹成功! %d→%d 重置阈值=%d\n", initial_ammo_, ammo, default_thr);
    return BT::NodeStatus::SUCCESS;
  }

  // 检查超时
  int timeout_s = 30;
  getInput("timeout_s", timeout_s);
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

  // 每5秒打印一次倒计时
  auto now = std::chrono::steady_clock::now();
  if (now - last_log_time_ >= std::chrono::seconds(5)) {
    last_log_time_ = now;
    std::printf("[HoldForSupplyAmmoTick] 进入补弹倒计时 | 已等待 %lds/%ds 当前弹药=%d 初始=%d\n",
                static_cast<long>(elapsed_s), timeout_s, ammo, initial_ammo_);
  }

  if (elapsed >= std::chrono::seconds(timeout_s)) {
    // 超时：降级阈值
    int fail_count = root_bb->get<int>("supply.fail_count");
    ++fail_count;
    root_bb->set<int>("supply.fail_count", fail_count);

    if (fail_count == 1) {
      int next = std::max(initial_ammo_ / 2, 1);
      root_bb->set<int>("supply.next_threshold", next);
      std::printf("[HoldForSupplyAmmoTick] 超时! 第%d次失败 → 新阈值=%d\n", fail_count, next);
    } else if (fail_count == 2) {
      root_bb->set<int>("supply.next_threshold", 1);
      std::printf("[HoldForSupplyAmmoTick] 超时! 第%d次失败 → 新阈值=1\n", fail_count);
    } else {
      root_bb->set<int>("supply.next_threshold", -1);
      std::printf("[HoldForSupplyAmmoTick] 超时! 第%d次失败 → 放弃补弹\n", fail_count);
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
