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
  int current_posture = 3, buff_cool = 0, buff_defense = 0, buff_vuln = 0, ammo = 300;

  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("has_target", has_target);
  getInput("base_threat", base_threat);
  getInput("is_disengaged", disengaged);
  getInput("stage_elapsed_time", elapsed);
  getInput("current_posture", current_posture);
  getInput("buff_cool_value", buff_cool);
  getInput("buff_defense_pct", buff_defense);
  getInput("buff_vulnerability_pct", buff_vuln);
  getInput("ammo_allow", ammo);

  int posture = 3;  // 默认移动
  const double hp_ratio = (hp_max > 0) ? static_cast<double>(hp) / hp_max : 1.0;

  // ── 强制防御条件 ──
  // 被易伤标记（vulnerability > 0）：受到的伤害被放大，必须防御
  if (buff_vuln > 0) {
    posture = 2;
  }
  // 弹药为零：无法输出，切移动保命
  else if (ammo <= 0) {
    posture = 3;
  }
  // 基地受威胁 → 防御
  else if (base_threat) {
    posture = 2;
  }
  // 血量低于 30% → 防御
  else if (hp_ratio < 0.3) {
    posture = 2;
  }
  // 热量超限 → 防御（减少输出避免超限惩罚）
  else if (heat > heat_high) {
    posture = 2;
  }
  // ── 进攻条件 ──
  // 有防御增益（defense > 0）且有目标 → 可以更激进进攻
  else if (has_target && buff_defense > 0) {
    posture = 1;
  }
  // 有冷却增益（cool > 0）且有目标 → 输出能力增强，进攻
  else if (has_target && buff_cool > 0 && hp_ratio > 0.4) {
    posture = 1;
  }
  // 有目标且血量 > 50% → 进攻
  else if (has_target && hp_ratio > 0.5) {
    posture = 1;
  }
  // ── 移动/默认 ──
  // 脱战状态 → 移动
  else if (disengaged) {
    posture = 3;
  }
  // 比赛后半段 (>180s) 无目标 → 进攻（增加裁判系统的允许发弹量获取）
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
