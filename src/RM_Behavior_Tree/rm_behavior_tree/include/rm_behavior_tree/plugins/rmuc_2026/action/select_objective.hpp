#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_OBJECTIVE_HPP_

#include <string>
#include <cmath>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "sp_msgs/msg/rmuc_game_status.hpp"

namespace rm_behavior_tree
{
/// 根据比赛局势选择下一个战略目标点（中心高地/梯形高地/敌方堡垒等）
class SelectObjectiveAction : public BT::SyncActionNode
{
public:
  SelectObjectiveAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      BT::InputPort<sp_msgs::msg::RMUCGameStatus>("game_status"),
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<int>("hp_cur"), BT::InputPort<int>("hp_max"),
      BT::InputPort<bool>("outpost_alive"),
      BT::InputPort<bool>("conservative_mode_enable", false, "保守模式开关"),
      BT::InputPort<int>("cap_sustain_time"),
      BT::InputPort<double>("cap_outpost_x"),
      BT::InputPort<double>("cap_outpost_y"),
      BT::InputPort<double>("central_highland_x"),
      BT::InputPort<double>("central_highland_y"),
      BT::InputPort<double>("ladder_highland_x"),
      BT::InputPort<double>("ladder_highland_y"),
      BT::InputPort<double>("fortress_area_x"),
      BT::InputPort<double>("fortress_area_y"),
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y"),
      BT::OutputPort<std::string>("objective_name")};
  }
  BT::NodeStatus tick() override;

private:
  bool cap_timer_started_{false};
  bool cap_timer_done_{false};
  std::chrono::steady_clock::time_point cap_timer_start_{};
};
}  // namespace rm_behavior_tree
#endif
