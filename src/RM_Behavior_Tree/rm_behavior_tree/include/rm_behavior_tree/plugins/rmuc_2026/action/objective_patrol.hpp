#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__OBJECTIVE_PATROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__OBJECTIVE_PATROL_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/// 目标巡逻节点：根据当前战略目标复用巡逻逻辑
/// - 目标是 CENTRAL_HIGHLAND 且 out_alive_patrol_enable=true → [中央高地, 存活巡逻点...] 循环
/// - 其他非 TRAPEZOIDAL_HIGHLAND 目标 → 只输出战略目标坐标
/// - 目标是 TRAPEZOIDAL_HIGHLAND → 根据 destroyed_outpost_ladder_enable 决定 [梯形高地, 堡垒区] 或 [堡垒区]
/// - 巡逻开启时追加普通巡逻点，形成 [梯形高地, 堡垒区, wpt0, wpt1, ...]
class ObjectivePatrolAction : public BT::SyncActionNode
{
public:
  ObjectivePatrolAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("objective_x"),
      BT::InputPort<double>("objective_y"),
      BT::InputPort<std::string>("objective_name"),
      BT::InputPort<double>("fortress_area_x"),
      BT::InputPort<double>("fortress_area_y"),
      BT::InputPort<bool>("patrol_enable", false, "是否启用巡逻"),
      BT::InputPort<bool>("destroyed_outpost_ladder_enable", true, "前哨站被毁且巡逻关闭时是否加入梯形高地"),
      BT::InputPort<std::string>("patrol_waypoints", "", "巡逻点 \"x1,y1;x2,y2;...\""),
      BT::InputPort<bool>("out_alive_patrol_enable", false, "前哨站存活时是否启用中央高地巡逻"),
      BT::InputPort<std::string>("out_alive_patrol_waypoints", "", "前哨站存活巡逻点 \"x1,y1;x2,y2;...\""),
      BT::InputPort<int>("ladder_time", 5000, "梯形高地停留时间(ms)"),
      BT::InputPort<int>("fortress_time", 5000, "堡垒区停留时间(ms)"),
      BT::InputPort<int>("patrol_hold_ms", 5000, "普通巡逻点/前哨站存活巡逻停留时间(ms)"),
      BT::InputPort<double>("arrive_radius", 0.5, "到达判定半径(m)"),
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y"),
      BT::OutputPort<bool>("goal_changed")};
  }

  BT::NodeStatus tick() override;

private:
  struct Pt
  {
    double x;
    double y;
    int hold_ms;
    const char * label;
  };

  static std::vector<Pt> parseWaypoints(const std::string & s);

  std::vector<Pt> cycle_;       // [objective, wpt0, wpt1, ...]
  int current_idx_{0};
  bool was_arrived_{false};
  std::chrono::steady_clock::time_point arrived_time_;
  int hold_ms_cache_{0};
  std::string last_waypoints_str_;
  double last_obj_x_{0}, last_obj_y_{0};
};

}  // namespace rm_behavior_tree
#endif
