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
      BT::OutputPort<std::string>("topic_rfid_status", "rfid status topic"),
      BT::OutputPort<std::string>("topic_robot_pose", "robot pose topic"),
      BT::OutputPort<double>("home_x"), BT::OutputPort<double>("home_y"),
      BT::OutputPort<double>("supply_zone_x"), BT::OutputPort<double>("supply_zone_y"),
      BT::OutputPort<double>("base_buff_x"), BT::OutputPort<double>("base_buff_y"),
      BT::OutputPort<double>("outpost_buff_x"), BT::OutputPort<double>("outpost_buff_y"),
      BT::OutputPort<double>("fortress_ally_x"), BT::OutputPort<double>("fortress_ally_y"),
      BT::OutputPort<double>("fortress_enemy_x"), BT::OutputPort<double>("fortress_enemy_y"),
      BT::OutputPort<double>("central_highland_x"), BT::OutputPort<double>("central_highland_y"),
      BT::OutputPort<double>("ladder_highland_x"), BT::OutputPort<double>("ladder_highland_y"),
      BT::OutputPort<double>("defend_anchor_x"), BT::OutputPort<double>("defend_anchor_y"),
      BT::OutputPort<double>("patrol_wpt_0_x"), BT::OutputPort<double>("patrol_wpt_0_y"),
      BT::OutputPort<double>("patrol_wpt_1_x"), BT::OutputPort<double>("patrol_wpt_1_y"),
      BT::OutputPort<double>("patrol_wpt_2_x"), BT::OutputPort<double>("patrol_wpt_2_y"),
      BT::OutputPort<double>("arrive_radius"),
      BT::OutputPort<int>("hp_low"),
      BT::OutputPort<int>("hp_safe"),
      BT::OutputPort<int>("heat_high"),
      BT::OutputPort<int>("ammo_low"),
      BT::OutputPort<int>("ammo_target"),
      BT::OutputPort<int>("base_deficit_for_fortress"),
      BT::OutputPort<double>("enemy_near_base_radius"),
      BT::OutputPort<int>("objective_hold_ms"),
      BT::OutputPort<int>("objective_hold_ms")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
