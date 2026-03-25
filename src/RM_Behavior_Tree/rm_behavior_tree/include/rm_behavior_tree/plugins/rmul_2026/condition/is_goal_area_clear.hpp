#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_AREA_CLEAR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_AREA_CLEAR_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * 评估目标点区域是否可安全通行。
 *
 * 算法（宽松评估，只拦截"去了肯定被困死"的情况）：
 *   1. 检查目标点 + 8个邻居（间距 check_radius）的 costmap 代价
 *   2. 如果目标点本身 cost < 253（非致命）→ SUCCESS
 *   3. 否则如果至少 min_free_neighbors 个邻居 cost < free_threshold → SUCCESS
 *   4. 否则 FAILURE（目标区域被完全封死）
 *
 * 返回：SUCCESS=可以前往  FAILURE=目标区域不可通行
 */
class IsGoalAreaClearCondition : public BT::ConditionNode
{
public:
  IsGoalAreaClearCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw",
                                 "costmap topic"),
      BT::InputPort<double>("goal_x", "goal x"),
      BT::InputPort<double>("goal_y", "goal y"),
      BT::InputPort<double>("check_radius", 0.3, "neighbor check radius in meters"),
      BT::InputPort<int>("min_free_neighbors", 2,
                          "min free neighbors to consider area passable"),
      BT::InputPort<std::uint8_t>("free_threshold", 200,
                                   "cost below this is considered free"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  std::string subscribed_topic_;

  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  uint8_t getCost(double world_x, double world_y) const;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_AREA_CLEAR_HPP_
