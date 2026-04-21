#include "rm_behavior_tree/plugins/rmuc_2026/action/init_sentry_config.hpp"

namespace rm_behavior_tree
{

InitSentryConfigAction::InitSentryConfigAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus InitSentryConfigAction::tick()
{
  if (loaded_once_) {
    return BT::NodeStatus::SUCCESS;
  }

  // 话题名称 (字符串默认值)
  auto setStr = [this](const char * key, const std::string & fallback) {
    std::string v = fallback;
    getInput(key, v);
    setOutput(key, v);
  };
  setStr("topic_game_status", "game_status");
  setStr("topic_robot_status", "robot_status");
  setStr("topic_rfid_status", "rfid_status");
  setStr("topic_robot_pose", "robot_position");

  // 坐标参数 (double, 默认 0.0)
  for (auto * k : {"home_x","home_y","supply_zone_x","supply_zone_y",
                    "base_buff_x","base_buff_y","outpost_buff_x","outpost_buff_y",
                    "fortress_ally_x","fortress_ally_y",
                    "central_highland_x","central_highland_y",
                    "ladder_highland_x","ladder_highland_y",
                    "defend_anchor_x","defend_anchor_y",
                    "central_highland_left_x","central_highland_left_y",
                    "ramp_jump_x","ramp_jump_y"})
  {
    double v = 0.0;
    getInput(k, v);
    setOutput(k, v);
  }

  // 阈值参数 (通过 getInput 读取，支持 YAML 覆盖)
  auto setDouble = [this](const char * key, double fallback) {
    double v = fallback;
    getInput(key, v);
    setOutput(key, v);
  };
  auto setInt = [this](const char * key, int fallback) {
    int v = fallback;
    getInput(key, v);
    setOutput(key, v);
  };

  setDouble("arrive_radius", 1.0);
  setDouble("enemy_near_base_radius", 2.0);

  setInt("hp_low", 180);
  setInt("hp_safe", 280);
  setInt("ammo_low", 80);
  setInt("objective_hold_ms", 12000);
  setInt("patrol_hold_ms", 5000);

  // 巡逻参数 (从黑板直接读取，由 rm_behavior_tree.cpp 注入)
  bool patrol_enable = false;
  getInput("patrol_enable", patrol_enable);
  setOutput("patrol_enable", patrol_enable);

  std::string patrol_waypoints;
  getInput("patrol_waypoints", patrol_waypoints);
  setOutput("patrol_waypoints", patrol_waypoints);

  loaded_once_ = true;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::InitSentryConfigAction>("InitSentryConfig");
}
