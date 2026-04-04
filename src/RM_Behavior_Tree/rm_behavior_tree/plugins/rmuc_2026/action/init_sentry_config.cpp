#include "rm_behavior_tree/plugins/rmuc_2026/action/init_sentry_config.hpp"

namespace rm_behavior_tree
{

InitSentryConfigAction::InitSentryConfigAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus InitSentryConfigAction::tick()
{
  // 所有端口均带有 XML 默认值，tick 时将默认值写入黑板
  // BT.CPP 会自动将 OutputPort 的 default 值写入对应黑板键
  // 此处显式 setOutput 以确保黑板已初始化（覆盖 XML 默认值即可）
  auto set = [this](const char * key, auto val) {
    setOutput(key, val);
  };
  set("topic_game_status", std::string("game_status"));
  set("topic_robot_status", std::string("robot_status"));
  set("topic_rfid_status", std::string("rfid_status"));
  set("topic_robot_pose", std::string("robot_position"));
  set("topic_radar_tracks", std::string("radar/enemy_tracks"));

  // 坐标默认为 0，由 XML 参数或上层设置覆盖
  for (auto * k : {"home_x","home_y","supply_x","supply_y",
                    "base_buff_x","base_buff_y","outpost_buff_x","outpost_buff_y",
                    "fortress_ally_x","fortress_ally_y","fortress_enemy_x","fortress_enemy_y",
                    "central_highland_x","central_highland_y",
                    "ladder_highland_x","ladder_highland_y",
                    "defend_anchor_x","defend_anchor_y",
                    "patrol_wpt_0_x","patrol_wpt_0_y",
                    "patrol_wpt_1_x","patrol_wpt_1_y",
                    "patrol_wpt_2_x","patrol_wpt_2_y"})
  {
    double v = 0.0;
    getInput(k, v);
    setOutput(k, v);
  }
  set("arrive_radius", 0.35);
  set("hp_critical", 80);
  set("hp_low", 180);
  set("hp_safe", 280);
  set("heat_high", 210);
  set("heat_critical", 245);
  set("ammo_low", 80);
  set("ammo_target", 300);
  set("base_deficit_for_fortress", 500);
  set("enemy_near_base_radius", 2.0);
  set("objective_hold_ms", 12000);
  set("combat_fire_burst_ms", 180);
  set("combat_fire_pause_ms", 120);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::InitSentryConfigAction>("InitSentryConfig");
}
