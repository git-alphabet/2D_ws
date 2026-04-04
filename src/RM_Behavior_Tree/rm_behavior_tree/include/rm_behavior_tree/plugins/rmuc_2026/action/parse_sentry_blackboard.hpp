#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__PARSE_SENTRY_BLACKBOARD_HPP_

#include <string>
#include <memory>
#include <cmath>
#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmuc_game_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"
#include "rm_decision_interfaces/msg/rmuc_enemy_tracks.hpp"

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
      // inputs (原始消息 — 拆分后各自独立类型)
      BT::InputPort<rm_decision_interfaces::msg::RMUCGameStatus>("game_status"),
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("robot_status"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<std::uint64_t>("now_ms"),
      // outputs (派生变量)
      BT::OutputPort<int>("stage_remain_time"),
      BT::OutputPort<int>("stage_elapsed_time"),
      BT::OutputPort<int>("hp_cur"),
      BT::OutputPort<int>("hp_max"),
      BT::OutputPort<int>("heat_cur"),
      BT::OutputPort<int>("ammo_allow"),
      BT::OutputPort<int>("ammo_left"),
      BT::OutputPort<int>("base_hp_cur"),
      BT::OutputPort<int>("base_hp_max"),
      BT::OutputPort<bool>("outpost_alive"),
      BT::OutputPort<bool>("is_dead"),
      BT::OutputPort<bool>("is_weak"),
      BT::OutputPort<bool>("is_disengaged"),
      BT::OutputPort<int>("disengage_countdown"),
      BT::OutputPort<bool>("can_remote_heal"),
      BT::OutputPort<bool>("can_remote_ammo"),
      BT::OutputPort<int>("team_coins"),
      BT::OutputPort<bool>("has_target"),
      BT::OutputPort<std::string>("best_target"),
      BT::OutputPort<bool>("base_threat"),
      BT::OutputPort<bool>("fortress_threat")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
