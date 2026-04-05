#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__OBJECTIVE_PATROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__OBJECTIVE_PATROL_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/// 目标巡逻节点：在战略目标点与巡逻点之间循环
/// - 巡逻关闭 或 目标不是 TRAPEZOIDAL_HIGHLAND → 只输出战略目标坐标
/// - 巡逻开启 + 目标是 TRAPEZOIDAL_HIGHLAND → 在 [目标点, wpt0, wpt1, ...] 之间周期循环
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
      BT::InputPort<bool>("patrol_enable", false, "是否启用巡逻"),
      BT::InputPort<std::string>("patrol_waypoints", "", "巡逻点 \"x1,y1;x2,y2;...\""),
      BT::InputPort<int>("patrol_hold_ms", 5000, "到达巡逻点后停留时间(ms)"),
      BT::InputPort<double>("arrive_radius", 0.5, "到达判定半径(m)"),
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y")};
  }

  BT::NodeStatus tick() override;

private:
  struct Pt { double x; double y; };

  static std::vector<Pt> parseWaypoints(const std::string & s);

  std::vector<Pt> cycle_;       // [objective, wpt0, wpt1, ...]
  int current_idx_{0};
  bool was_arrived_{false};
  std::chrono::steady_clock::time_point arrived_time_;
  std::string last_waypoints_str_;
  double last_obj_x_{0}, last_obj_y_{0};
};

}  // namespace rm_behavior_tree
#endif
