#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_POSTURE_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 根据血量、枪管热量、目标、基地威胁等状态决定机器人姿态 (1=进攻 2=防御 3=移动)
class DecidePostureAction : public BT::SyncActionNode
{
public:
  DecidePostureAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_high"),
      BT::InputPort<bool>("has_target"),
      BT::InputPort<bool>("base_threat"),
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<std::uint64_t>("now_ms"),
      BT::InputPort<int>("current_posture"),
      BT::InputPort<int>("buff_cool_value"),
      BT::InputPort<int>("buff_defense_pct"),
      BT::InputPort<int>("buff_vulnerability_pct"),
      BT::InputPort<int>("ammo_allow"),
      BT::OutputPort<int>("posture_out")};
  }
  BT::NodeStatus tick() override;

private:
  int last_posture_{3};
  std::uint64_t last_switch_ms_{0};
  bool posture_initialized_{false};
  bool crisis_override_active_{false};
};
}  // namespace rm_behavior_tree
#endif
