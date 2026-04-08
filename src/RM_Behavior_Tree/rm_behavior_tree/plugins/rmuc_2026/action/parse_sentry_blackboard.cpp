#include "rm_behavior_tree/plugins/rmuc_2026/action/parse_sentry_blackboard.hpp"

namespace rm_behavior_tree
{

ParseSentryBlackboardAction::ParseSentryBlackboardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ParseSentryBlackboardAction::tick()
{
  // ── 比赛阶段 (GameStatus) ──
  auto game_msg = getInput<sp_msgs::msg::RMUCGameStatus>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  // ── 机器人状态 (RobotStatus) ──
  auto robot_ptr = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("hp_max", static_cast<int>(r.max_hp));
    setOutput("heat_cur", static_cast<int>(r.shooter_heat));
    setOutput("ammo_allow", static_cast<int>(r.ammo_allow));
    setOutput("ammo_left", static_cast<int>(r.ammo_left));
    setOutput("base_hp_cur", static_cast<int>(r.base_hp_cur));
    setOutput("base_hp_max", static_cast<int>(r.base_hp_max));
    setOutput("outpost_alive", r.outpost_alive);
    setOutput("is_dead", r.current_hp <= 0);
    setOutput("can_remote_heal", r.can_remote_heal);
    setOutput("can_remote_ammo", r.can_remote_ammo);
    setOutput("team_coins", static_cast<int>(r.team_coins));
    setOutput("has_target", r.is_detect_enemy);
  }

  auto radar_tracks = getInput<sp_msgs::msg::RMUCEnemyTracks>("radar_tracks");
  if (radar_tracks) {
    const bool has_radar_target = radar_tracks->enemy_count > 0;
    if (has_radar_target) {
      setOutput("has_target", true);
    }
  }

  // ── 哨兵决策状态 (SentryDecisionStatus 0x020D) ──
  auto sds = getInput<sp_msgs::msg::RMUCSentryDecisionStatus>("sentry_decision_status");
  if (sds) {
    setOutput("can_free_respawn", sds->can_free_respawn);
    setOutput("can_instant_respawn", sds->can_instant_respawn);
    setOutput("instant_respawn_cost", static_cast<int>(sds->instant_respawn_cost));
    setOutput("current_posture", static_cast<int>(sds->current_posture));
    setOutput("remote_ammo_count", static_cast<int>(sds->remote_ammo_count));
    setOutput("remote_heal_count", static_cast<int>(sds->remote_heal_count));
    setOutput("exchanged_ammo_total", static_cast<int>(sds->exchanged_ammo_total));
    setOutput("can_activate_energy", sds->can_activate_energy);
  }

  // ── 增益状态 (RobotBuff) ──
  auto buff = getInput<sp_msgs::msg::RMUCRobotBuff>("robot_buff");
  if (buff) {
    setOutput("buff_heal_rate", static_cast<int>(buff->heal_rate));
    setOutput("buff_cool_value", static_cast<int>(buff->cool_value));
    setOutput("buff_defense_pct", static_cast<int>(buff->defense_pct));
    setOutput("buff_vulnerability_pct", static_cast<int>(buff->vulnerability_pct));
    setOutput("buff_attack_pct", static_cast<int>(buff->attack_pct));
  }

  // ── 弹丸配额 (ProjectileAllowance) ──
  auto pa = getInput<sp_msgs::msg::RMUCProjectileAllowance>("projectile_allowance");
  if (pa) {
    setOutput("fortress_ammo", static_cast<int>(pa->fortress_ammo));
  }

  // ── 场地状态 (FieldStatus) ──
  auto fs = getInput<sp_msgs::msg::RMUCFieldStatus>("field_status");
  if (fs) {
    setOutput("field_central_highland", static_cast<int>(fs->central_highland));
    setOutput("field_ladder_highland", static_cast<int>(fs->ladder_highland));
    setOutput("field_fortress", static_cast<int>(fs->fortress));
    setOutput("field_outpost_buff", static_cast<int>(fs->outpost_buff));
    setOutput("field_base_buff", fs->base_buff);
    setOutput("field_small_energy", static_cast<int>(fs->small_energy_status));
    setOutput("field_big_energy", static_cast<int>(fs->big_energy_status));
  }

  // ── 敌方易伤标记 (EnemyMark) ──
  auto em = getInput<sp_msgs::msg::RMUCEnemyMark>("enemy_mark");
  if (em) {
    setOutput("enemy_hero_vuln", em->enemy_hero_vuln);
    setOutput("enemy_engi_vuln", em->enemy_engi_vuln);
    setOutput("enemy_infantry3_vuln", em->enemy_infantry3_vuln);
    setOutput("enemy_infantry4_vuln", em->enemy_infantry4_vuln);
    setOutput("enemy_sentry_vuln", em->enemy_sentry_vuln);
  }

  // ── 队伍血量 (TeamHP) ──
  auto th = getInput<sp_msgs::msg::RMUCTeamHP>("team_hp");
  if (th) {
    setOutput("team_outpost_hp", static_cast<int>(th->outpost_hp));
    setOutput("team_base_hp", static_cast<int>(th->base_hp));
  }

  // 先把基地半血条件前移到黑板派生层，统一由 base_threat 表示。
  bool base_threat = false;
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    base_threat = (r.base_hp_max > 0 && r.base_hp_cur < r.base_hp_max / 2);
  }
  setOutput("base_threat", base_threat);
  setOutput("fortress_threat", false);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
