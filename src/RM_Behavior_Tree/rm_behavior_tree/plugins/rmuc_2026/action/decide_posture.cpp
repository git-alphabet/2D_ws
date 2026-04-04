#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_posture.hpp"

namespace rm_behavior_tree
{

DecidePostureAction::DecidePostureAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecidePostureAction::tick()
{
  int hp = 400, hp_max = 400, heat = 0, heat_high = 210, elapsed = 0;
  bool has_target = false, base_threat = false, disengaged = false;

  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("has_target", has_target);
  getInput("base_threat", base_threat);
  getInput("is_disengaged", disengaged);
  getInput("stage_elapsed_time", elapsed);

  int posture = 3;  // 默认移动

  // 基地受威胁 → 防御
  if (base_threat) {
    posture = 2;
  }
  // 血量低于 30% 或 热量高 → 防御
  else if (hp_max > 0 && hp < hp_max * 0.3) {
    posture = 2;
  } else if (heat > heat_high) {
    posture = 2;
  }
  // 有目标且血量 > 50% → 进攻
  else if (has_target && hp > hp_max * 0.5) {
    posture = 1;
  }
  // 脱战状态 → 移动
  else if (disengaged) {
    posture = 3;
  }
  // 比赛后半段 (>180s) 默认进攻
  else if (elapsed > 180) {
    posture = 1;
  }

  setOutput("posture_out", posture);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecidePostureAction>("DecidePosture");
}
