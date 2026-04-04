#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_NAV_GOAL_FROM_CONFIG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_NAV_GOAL_FROM_CONFIG_HPP_

#include <string>
#include <cmath>
#include <algorithm>  // std::clamp

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{

class SetNavGoalFromConfigAction : public BT::SyncActionNode
{
public:
  SetNavGoalFromConfigAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      // 输入：配置目标点（补给区坐标）
      BT::InputPort<double>("cfg_x"),
      BT::InputPort<double>("cfg_y"),

      // 输出：导航目标点（调试/给其它节点）
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y"),

      // 输出：给 SendGoal 直接发布
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  // 1) 安全限幅：防止配置异常导致目标极端（可按你们场地坐标系后续调整）
  static constexpr double MAP_LIMIT_M = 50.0;

  // 2) 当 cfg 丢失/非法时：是否允许用默认点（不建议，默认 false：宁可不更新也别拉去原点）
  static constexpr bool ALLOW_FALLBACK_TO_ZERO = false;

  // 3) 内部缓存：上一次有效目标（防抖/抗异常）
  bool last_goal_valid_ = false;
  double last_goal_x_ = 0.0;
  double last_goal_y_ = 0.0;

  static bool isFinite(double v) { return std::isfinite(v); }

  static double clampToMap(double v)
  {
    return std::clamp(v, -MAP_LIMIT_M, MAP_LIMIT_M);
  }

  static geometry_msgs::msg::PoseStamped makePose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;

    // 默认朝向：单位四元数
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    return p;
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_NAV_GOAL_FROM_CONFIG_HPP_
