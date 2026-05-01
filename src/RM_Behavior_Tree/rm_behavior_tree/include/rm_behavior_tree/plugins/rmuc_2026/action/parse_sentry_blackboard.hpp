#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <chrono>
#include <utility>
#include <vector>
#include "behaviortree_cpp/action_node.h"
#include "sp_msgs/msg/rmuc_game_status.hpp"
#include "sp_msgs/msg/rmuc_robot_status.hpp"
#include "sp_msgs/msg/rmuc_enemy_tracks.hpp"
#include "sp_msgs/msg/rmuc_team_hp.hpp"

namespace rm_behavior_tree
{
struct RmucSemanticZone
{
  std::string name;
  std::string type;
  std::vector<std::pair<double, double>> vertices;
};

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
      BT::InputPort<std::string>("semantic_zones_file"),
      BT::InputPort<std::string>("semantic_ignore_enemy_zone_type", "speed_bump"),

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
      BT::OutputPort<bool>("is_detect_enemy"),

      // ── outputs: 占位/兼容 ──
      BT::OutputPort<bool>("base_threat"),

      // ── outputs: 脱战派生 ──
      BT::OutputPort<bool>("is_disengaged")};
  }

  BT::NodeStatus tick() override;

private:
  void loadSemanticZones(const std::string & yaml_path);
  void resetSemanticZonesState();
  void logSemanticZonesLoadErrorOnce(const std::string & yaml_path, const std::string & error);
  bool pointInPolygon(
    double x, double y,
    const std::vector<std::pair<double, double>> & poly) const;
  bool isInSemanticZone(double x, double y, const std::string & zone_type);

  bool base_threat_latched_{false};
  int last_base_hp_{-1};
  bool logged_missing_base_config_{false};
  bool logged_missing_base_radius_{false};
  bool logged_missing_base_calm_timeout_{false};
  std::chrono::steady_clock::time_point last_base_threat_trigger_log_{};
  bool has_base_threat_trigger_log_{false};

  // 基地威胁自动解除：危机模式下连续无敌人+基地不掉血超过30s→自动解除
  std::chrono::steady_clock::time_point base_threat_calm_start_{};
  bool base_threat_calm_tracking_{false};

  // 脱战检测
  uint16_t last_shooter_heat_{0};
  uint16_t last_current_hp_{0};
  uint64_t last_activity_ms_{0};
  bool disengage_initialized_{false};

  std::vector<RmucSemanticZone> semantic_zones_;
  bool semantic_zones_loaded_{false};
  bool semantic_zones_load_attempted_{false};
  std::string semantic_zones_file_;
  std::string semantic_zones_last_error_;
  bool semantic_enemy_override_active_{false};
};
}  // namespace rm_behavior_tree

#endif
