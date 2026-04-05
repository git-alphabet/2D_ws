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

  const double hp_ratio = (hp_max > 0) ? static_cast<double>(hp) / hp_max : 1.0;

  // ═══════ 综合评分系统 ═══════
  // 三姿态各自累加评分，最高分胜出

  // ── 攻击评分 (posture=1) ──
  int score_attack = 5;                                 // 基础分
  if (has_target)          score_attack += 35;           // 有目标
  if (hp_ratio > 0.5)      score_attack += 20;           // 血量健康
  else if (hp_ratio > 0.3) score_attack += 8;            // 血量尚可
  if (buff_defense > 0)    score_attack += 15;           // 防御增益护体
  if (buff_cool > 0)       score_attack += 12;           // 冷却增益增加DPS
  if (elapsed > 180)       score_attack += 18;           // 后半场争取发弹配额
  if (ammo <= 0)           score_attack = 0;             // 无弹药强制归零

  // ── 防御评分 (posture=2) ──
  int score_defense = 8;                                 // 基础分
  if (buff_vuln > 0)       score_defense += 50;          // 易伤标记
  if (base_threat)         score_defense += 30;          // 基地受威胁
  if (hp_ratio < 0.3)      score_defense += 35;          // 血量危急
  else if (hp_ratio < 0.5) score_defense += 15;          // 血量偏低
  if (heat > heat_high)    score_defense += 25;          // 热量超限
  if (!has_target)         score_defense += 8;           // 无目标倾向防御

  // ── 移动评分 (posture=3) ──
  int score_move = 10;                                   // 基础分(默认倾向)
  if (ammo <= 0)           score_move += 50;             // 无弹药必须机动
  if (disengaged)          score_move += 20;             // 脱战状态
  if (!has_target)         score_move += 10;             // 无目标巡逻
  if (hp_ratio >= 0.3 && hp_ratio < 0.5) score_move += 5; // 中低血量灵活走位

  // ── 选择最高分姿态（平局优先防御 > 移动 > 攻击）──
  int posture = 3;
  int max_score = score_move;
  if (score_defense >= max_score) {
    posture = 2;
    max_score = score_defense;
  }
  if (score_attack > max_score) {
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
