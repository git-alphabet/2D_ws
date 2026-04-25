#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_NAV_TARGET_SUPPLY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_NAV_TARGET_SUPPLY_HPP_

#include <string>
#include <mutex>
#include <atomic>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 查询当前 Nav2 导航目标并与给定坐标比较。
 *
 * 内部订阅 goal_pose 话题（SendGoal 发布导航目标的同一话题），
 * 缓存最新目标位姿，tick 时与 (goal_x, goal_y) 比较。
 *
 * 返回：
 *  - SUCCESS  当前导航目标在 arrive_radius 范围内
 *  - FAILURE  无目标 或 不在范围内
 */
class IsNavTargetSupplyCondition : public BT::SyncActionNode
{
public:
  IsNavTargetSupplyCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("goal_x", "target X to compare against"),
      BT::InputPort<double>("goal_y", "target Y to compare against"),
      BT::InputPort<double>("arrive_radius", 1.5, "match radius"),
      BT::InputPort<std::string>("topic_name", "goal_pose", "Nav2 goal topic"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  std::mutex mtx_;
  geometry_msgs::msg::PoseStamped last_goal_;
  std::atomic<bool> has_goal_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_NAV_TARGET_SUPPLY_HPP_
