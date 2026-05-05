#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_SENTRY_CONFIG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__INIT_SENTRY_CONFIG_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 将所有静态配置参数输出到黑板，仅执行一次
/// 优先读取 XML 端口值（可由外部注入黑板覆盖），否则使用内置默认值
class InitSentryConfigAction : public BT::SyncActionNode
{
public:
  InitSentryConfigAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("topic_game_status", "game status topic"),
      BT::OutputPort<std::string>("topic_robot_status", "robot status topic"),
      BT::OutputPort<std::string>("topic_robot_pose", "robot pose topic"),
      BT::BidirectionalPort<double>("home_x"), BT::BidirectionalPort<double>("home_y"),
      BT::BidirectionalPort<double>("supply_zone_x"), BT::BidirectionalPort<double>("supply_zone_y"),
      BT::BidirectionalPort<double>("base_x"), BT::BidirectionalPort<double>("base_y"),
      BT::BidirectionalPort<double>("base_buff_x"), BT::BidirectionalPort<double>("base_buff_y"),
      BT::BidirectionalPort<double>("outpost_buff_x"), BT::BidirectionalPort<double>("outpost_buff_y"),
      BT::BidirectionalPort<double>("central_highland_x"), BT::BidirectionalPort<double>("central_highland_y"),
      BT::BidirectionalPort<double>("ladder_highland_x"), BT::BidirectionalPort<double>("ladder_highland_y"),
      BT::BidirectionalPort<double>("defend_anchor_x"), BT::BidirectionalPort<double>("defend_anchor_y"),
      BT::BidirectionalPort<double>("central_highland_left_x"), BT::BidirectionalPort<double>("central_highland_left_y"),
      BT::BidirectionalPort<double>("ramp_jump_x"), BT::BidirectionalPort<double>("ramp_jump_y"),
      BT::BidirectionalPort<double>("cap_outpost_x"), BT::BidirectionalPort<double>("cap_outpost_y"),
      BT::BidirectionalPort<double>("arrive_radius"),
      BT::BidirectionalPort<int>("hp_low"),
      BT::BidirectionalPort<int>("hp_safe"),
      BT::BidirectionalPort<int>("ammo_low"),
      BT::BidirectionalPort<int>("supply_wait_timeout_s"),
      BT::BidirectionalPort<double>("enemy_near_base_radius"),
      BT::BidirectionalPort<int>("base_threat_calm_timeout_ms"),
      BT::BidirectionalPort<int>("patrol_hold_ms"),
      BT::BidirectionalPort<bool>("patrol_enable"),
      BT::BidirectionalPort<std::string>("patrol_waypoints"),
      BT::BidirectionalPort<std::string>("semantic_zones_file"),
      BT::BidirectionalPort<std::string>("semantic_ignore_enemy_zone_type")};
  }
  BT::NodeStatus tick() override;

private:
  bool loaded_once_{false};
};
}  // namespace rm_behavior_tree
#endif
