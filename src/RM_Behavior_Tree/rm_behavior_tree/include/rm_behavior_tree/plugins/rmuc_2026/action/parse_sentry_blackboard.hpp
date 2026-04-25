#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "sp_msgs/msg/rmuc_game_status.hpp"
#include "sp_msgs/msg/rmuc_robot_status.hpp"
#include "sp_msgs/msg/rmuc_enemy_tracks.hpp"
#include "sp_msgs/msg/rmuc_team_hp.hpp"

namespace rm_behavior_tree
{
/// 从黑板读取 RMUC 原始消息，解析出派生状态变量写入黑板
class ParseSentryBlackboardAction : public BT::SyncActionNode
{
public:
  ParseSentryBlackboardAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // ── inputs: 原始消息 ──
      BT::InputPort<sp_msgs::msg::RMUCGameStatus>("game_status"),
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status"),
      BT::InputPort<sp_msgs::msg::RMUCEnemyTracks>("radar_tracks"),
      BT::InputPort<sp_msgs::msg::RMUCTeamHP>("team_hp"),
      BT::InputPort<double>("base_x"),
      BT::InputPort<double>("base_y"),
      BT::InputPort<double>("defend_anchor_x"),
      BT::InputPort<double>("defend_anchor_y"),
      BT::InputPort<double>("enemy_near_base_radius"),
      BT::InputPort<int>("base_threat_calm_timeout_ms"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<std::uint64_t>("now_ms"),

      // ── outputs: GameStatus 派生 ──
      BT::OutputPort<int>("stage_remain_time"),
      BT::OutputPort<int>("stage_elapsed_time"),

      // ── outputs: RobotStatus 派生 ──
      BT::OutputPort<int>("hp_cur"),
      BT::OutputPort<int>("ammo_allow"),
      BT::OutputPort<int>("base_hp_cur"),
      BT::OutputPort<bool>("outpost_alive"),
      BT::OutputPort<bool>("is_dead"),
      BT::OutputPort<bool>("has_target"),

      // ── outputs: 占位/兼容 ──
      BT::OutputPort<bool>("base_threat"),

      // ── outputs: 脱战派生 ──
      BT::OutputPort<bool>("is_disengaged")};
  }

  BT::NodeStatus tick() override;

private:
  bool base_threat_latched_{false};
  int last_base_hp_{-1};
  bool logged_missing_base_config_{false};
  bool logged_missing_base_radius_{false};
  bool logged_missing_base_calm_timeout_{false};

  // 基地威胁自动解除：危机模式下连续无敌人+基地不掉血超过30s→自动解除
  std::chrono::steady_clock::time_point base_threat_calm_start_{};
  bool base_threat_calm_tracking_{false};

  // 脱战检测
  uint16_t last_shooter_heat_{0};
  uint16_t last_current_hp_{0};
  uint64_t last_activity_ms_{0};
  bool disengage_initialized_{false};
};
}  // namespace rm_behavior_tree

#endif
