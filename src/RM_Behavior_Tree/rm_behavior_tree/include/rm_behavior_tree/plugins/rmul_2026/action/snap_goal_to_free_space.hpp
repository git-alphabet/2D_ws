#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SNAP_GOAL_TO_FREE_SPACE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SNAP_GOAL_TO_FREE_SPACE_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * 检查目标点是否在 costmap 障碍物（含膨胀层）内，
 * 若是则螺旋搜索最近可达点并输出修正后的目标。
 *
 * 输入：goal_x/goal_y 或 goal_pose
 * 输出：同名端口（原地修改），若目标已在自由空间则原样输出
 * 返回：SUCCESS=找到可达目标  FAILURE=搜索半径内无可达点
 */
class SnapGoalToFreeSpaceAction : public BT::SyncActionNode
{
public:
  SnapGoalToFreeSpaceAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw",
                                 "costmap topic to subscribe"),
      BT::InputPort<double>("max_search_radius", 1.0,
                            "spiral search radius in meters"),
      BT::InputPort<double>("search_step", 0.1,
                            "step size for spiral search in meters"),
      BT::InputPort<std::uint8_t>("lethal_threshold", 253,
                                  "cost >= this is treated as obstacle (INSCRIBED=253)"),
      BT::BidirectionalPort<double>("goal_x"),
      BT::BidirectionalPort<double>("goal_y"),
      BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("goal_pose")
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

  /// 查询网格代价值；越界返回 255 (LETHAL)
  uint8_t getCost(double world_x, double world_y) const;

  /// 螺旋搜索最近可达点
  bool spiralSearch(double cx, double cy, double max_radius, double step,
                    uint8_t threshold, double & out_x, double & out_y) const;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SNAP_GOAL_TO_FREE_SPACE_HPP_
