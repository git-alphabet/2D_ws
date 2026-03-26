#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_APPROACH_POINT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_APPROACH_POINT_HPP_

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
 * @brief 在目标点附近搜索代价最低的可达接近点
 *
 * 当目标点（如控制区）被障碍物覆盖(cost≥253)时，在目标点周围
 * search_radius 范围内搜索代价最低的可达点作为接近等待点。
 *
 * 搜索策略：
 *   - 从目标点为圆心做同心环搜索(0.3m~search_radius, step=0.15m)
 *   - 候选点条件: cost < cost_threshold(默认150) 且从机器人到候选点路径无致命障碍
 *   - 排序: 按 cost 升序(越低越好)，同 cost 按距目标越近优先
 *
 * 状态保持：一旦选定接近点，后续 tick 返回同一个点，直到：
 *   - 该点代价值升高(cost >= 235)
 *   - 机器人距该点 > 8.0m
 *   - 距上次 tick 超过 2s（树已切走）
 *
 * 返回: SUCCESS=找到接近点  FAILURE=无可用点
 */
class FindApproachPointAction : public BT::SyncActionNode
{
public:
  FindApproachPointAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw",
                                 "costmap topic to subscribe"),
      BT::InputPort<double>("robot_x", "robot current x"),
      BT::InputPort<double>("robot_y", "robot current y"),
      BT::InputPort<double>("goal_x", "original (blocked) goal x"),
      BT::InputPort<double>("goal_y", "original (blocked) goal y"),
      BT::InputPort<double>("search_radius", 4.5, "max search radius around goal"),
      BT::InputPort<double>("min_radius", 0.3, "min search radius around goal"),
      BT::InputPort<uint8_t>("cost_threshold", 150, "max acceptable cost for approach point"),
      BT::OutputPort<double>("approach_x", "approach point x"),
      BT::OutputPort<double>("approach_y", "approach point y"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose",
        "approach point as PoseStamped"),
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
  bool isPathClear(double ax, double ay, double bx, double by,
                   uint8_t lethal_threshold = 253, double sample_step = 0.1) const;

  // 状态保持
  bool has_committed_ = false;
  double committed_x_ = 0.0;
  double committed_y_ = 0.0;
  static constexpr double MAX_COMMIT_DISTANCE = 8.0;

  // 心跳检测
  bool last_tick_time_set_ = false;
  std::chrono::steady_clock::time_point last_tick_time_;

  void outputResult(double x, double y);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_APPROACH_POINT_HPP_
